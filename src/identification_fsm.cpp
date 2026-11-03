/**
 * @brief 统一调度两类辨识波形：Position 采集推力 → Offboard 回中 → 脉冲组 → 回中 → 下一组。
 * 全程以 FLU 角速度和高度闭环推力控制；只在回中时启用姿态/水平速度反馈。
 * 不解锁、不起飞；完成或保护触发后异步请求退出 Offboard，人工退出立即停止激励。
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <holydrone_identification/recenter_control.hpp>
#include <holydrone_identification/excitation_plan.hpp>
#include <chrono>
#include <future>
#include <sstream>
#include <stdexcept>

static double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class Identification {
  ros::NodeHandle nh_, priv_{"~"};
  ros::Publisher command_pub_, status_pub_, finished_pub_;
  ros::Subscriber state_sub_, thrust_sub_, pose_sub_, velocity_sub_, imu_sub_;
  ros::WallTimer timer_;
  std::vector<ros::ServiceServer> services_;
  std::future<bool> mode_request_;
  std::string mavros_, exciter_, exit_mode_, phase_ = "waiting", reason_;
  std::vector<identification::Group> plan_;
  size_t group_ = 0, pulse_ = 0;
  bool offboard_ = false, position_armed_ = false, locked_ = false;
  bool active_ = false, pulsing_ = false, exiting_ = false, complete_ = false;
  double hover_ = 0, sent_thrust_ = 0, thrust_base_ = 0;
  double state_time_ = 0, thrust_time_ = 0, pose_time_ = 0, velocity_time_ = 0;
  double stage_time_ = 0, stable_time_ = 0, last_request_ = 0, last_status_ = 0;
  ros::Time wave_start_;
  ros::Time position_since_, thrust_stamp_, pose_stamp_, velocity_stamp_, control_stamp_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero(), origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  double yaw_ = 0, origin_yaw_ = 0, cos_tilt_ = 1;
  double clock_lead_tolerance_ = 0;
  double angular_rate_ = 0, imu_time_ = 0, max_body_rate_ = .15;
  ros::Time imu_stamp_;
  double frequency_ = 200, pulse_hold_ = .35, settle_ = .2;
  double recovery_timeout_ = 8, height_tolerance_ = .2, speed_tolerance_ = .5, horizontal_tolerance_ = .75;
  double tilt_tolerance_ = 8, yaw_tolerance_ = 5, feedback_timeout_ = .5;
  double max_tilt_ = 45, max_height_ = 1, max_distance_ = 5, max_vz_ = 2;
  identification::AltitudeControl altitude_;
  identification::RecenterControl recenter_;

  void positive(const std::string& name, double& value) {
    priv_.param(name, value, value);
    if (!std::isfinite(value) || value <= 0) throw std::runtime_error("invalid parameter: "+name);
  }
  void config() {
    priv_.param<std::string>("mavros_namespace", mavros_, "/mavros");
    std::string mode;
    if (ros::Time::isSimTime() && nh_.getParam(mavros_+"/time/timesync_mode", mode) && mode == "PASSTHROUGH")
      clock_lead_tolerance_ = .008;

    priv_.param<std::string>("exciter", exciter_, "brct1order_excite");
    priv_.param<std::string>("exit_mode", exit_mode_, "POSCTL");
    if (exit_mode_ != "POSCTL" && exit_mode_ != "AUTO.LOITER")
      throw std::runtime_error("exit_mode must be POSCTL or AUTO.LOITER");
    const bool physics = exciter_ == "gazebo_physics_excite";
    const bool second = exciter_ == "brct2order_excite";
    if (!physics && !second && exciter_ != "brct1order_excite") throw std::runtime_error("unknown exciter");
    pulse_hold_ = physics ? .5 : .35;
    std::vector<double> rates = physics ? std::vector<double>{.3,.3,.25} : std::vector<double>{.2,.2,.25};
    priv_.param("rate_amplitudes", rates, rates);
    double thrust = physics ? .025 : .015;
    int repetitions = physics ? 2 : 3;
    priv_.param("repetitions", repetitions, repetitions);
    positive("frequency", frequency_); positive("pulse_hold", pulse_hold_); positive("thrust_amplitude", thrust);
    positive("recovery/max_body_rate_rad_s", max_body_rate_);
    positive("recovery/settle_s", settle_); positive("recovery/timeout_s", recovery_timeout_);
    positive("recovery/height_tolerance_m", height_tolerance_);
    positive("recovery/horizontal_tolerance_m", horizontal_tolerance_);
    positive("recovery/speed_tolerance_m_s", speed_tolerance_);
    positive("recovery/tilt_tolerance_deg", tilt_tolerance_);
    positive("recovery/yaw_tolerance_deg", yaw_tolerance_);
    positive("recovery/position_gain", recenter_.position_gain);
    positive("recovery/velocity_gain", recenter_.velocity_gain);
    positive("recovery/attitude_gain", recenter_.attitude_gain);
    positive("recovery/max_tilt_deg", recenter_.max_tilt_deg);
    positive("recovery/max_rate_rad_s", recenter_.max_rate);
    positive("altitude_control/kp", altitude_.kp); positive("altitude_control/kd", altitude_.kd);
    positive("altitude_control/ki", altitude_.ki);
    positive("altitude_control/acceleration_limit", altitude_.acceleration_limit);
    positive("altitude_control/integral_limit", altitude_.integral_limit);
    positive("altitude_control/min_thrust", altitude_.min_thrust);
    positive("altitude_control/max_thrust", altitude_.max_thrust);
    positive("protection/feedback_timeout_s", feedback_timeout_);
    positive("protection/max_tilt_deg", max_tilt_); positive("protection/max_height_error_m", max_height_);
    positive("protection/max_distance_m", max_distance_); positive("protection/max_vertical_speed_m_s", max_vz_);
    if (frequency_ < 20 || frequency_ > 500 || repetitions < 1 || rates.size() != 3 ||
        recovery_timeout_ <= settle_ || max_tilt_ >= 80 || recenter_.max_tilt_deg >= max_tilt_ ||
        tilt_tolerance_ >= max_tilt_ || yaw_tolerance_ >= 180 || horizontal_tolerance_ >= max_distance_ || height_tolerance_ >= max_height_ ||
        altitude_.min_thrust >= altitude_.max_thrust || altitude_.max_thrust > 1 || thrust >= .5)
      throw std::runtime_error("inconsistent excitation/control/protection limits");
    for (double r : rates) if (!std::isfinite(r) || r <= 0) throw std::runtime_error("invalid rate amplitude");
    const std::array<double, 3> a{{rates[0],rates[1],rates[2]}};
    std::vector<double> durations;
    if (second) durations = {.2, .35, .5};
    if (physics || second) priv_.getParam("pulse_durations_s", durations);
    for (double d : durations) if (!std::isfinite(d) || d < .1 || d > .75)
      throw std::runtime_error("pulse_durations_s must be within [0.1, 0.75] s");
    plan_ = physics ? identification::physicsPlan(a, thrust, repetitions, durations) :
      second ? identification::secondOrderPlan(a, thrust, repetitions, durations) :
               identification::firstOrderPlan(a, thrust, repetitions);
    bool sweep=false;
    priv_.param("yaw_sweep/enabled",sweep,false);
    if(sweep) {
      if(!physics) throw std::runtime_error("yaw sweep is only supported for gazebo_physics");
      std::vector<double> frequencies{.5,1.,2.,3.};
      int cycles=4; double amplitude=.25;
      priv_.param("yaw_sweep/frequencies_hz",frequencies,frequencies);
      priv_.param("yaw_sweep/cycles",cycles,cycles);
      priv_.param("yaw_sweep/amplitude_rad_s",amplitude,amplitude);
      plan_=identification::withYawSweep(plan_,frequencies,cycles,amplitude);
    }
    for (const auto& old : {"hover_thrust", "baseline_seconds", "recovery_seconds"})
      if (priv_.hasParam(old)) ROS_WARN("%s is no longer used; thrust is captured in POSCTL, recovery is feedback-gated", old);
  }
  bool motionFresh() const {
    return pose_time_ > 0 && velocity_time_ > 0 && imu_time_ > 0 &&
      steady()-imu_time_ < feedback_timeout_ &&
      steady()-pose_time_ < feedback_timeout_ && steady()-velocity_time_ < feedback_timeout_;
  }
  bool stateFresh() const { return state_time_ > 0 && steady()-state_time_ < 2; }
  bool ready(double max_age = .5) const {
    return stateFresh() && position_armed_ && thrust_time_ > 0 && steady()-thrust_time_ < max_age && motionFresh();
  }
  bool validStamp(const ros::Time& stamp, const ros::Time& previous) const {
    const double age = (ros::Time::now()-stamp).toSec();
    return stamp > previous && age >= -clock_lead_tolerance_ && age <= feedback_timeout_;
  }
  void pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const auto& p = msg->pose.position; const auto& q = msg->pose.orientation;
    Eigen::Vector3d v(p.x,p.y,p.z); Eigen::Quaterniond a(q.w,q.x,q.y,q.z);
    if (!v.allFinite() || !a.coeffs().allFinite() || a.squaredNorm() < .5 || a.squaredNorm() > 1.5 ||
        !validStamp(msg->header.stamp, pose_stamp_)) return;
    position_ = v; attitude_ = a.normalized();
    cos_tilt_ = identification::bound(1-2*(attitude_.x()*attitude_.x()+attitude_.y()*attitude_.y()), -1, 1);
    yaw_ = std::atan2(2*(attitude_.w()*attitude_.z()+attitude_.x()*attitude_.y()),
                     1-2*(attitude_.y()*attitude_.y()+attitude_.z()*attitude_.z()));
    pose_time_ = steady(); pose_stamp_ = msg->header.stamp;
  }
  void velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const auto& v = msg->twist.linear; Eigen::Vector3d value(v.x,v.y,v.z);
    if (!value.allFinite() || !validStamp(msg->header.stamp, velocity_stamp_)) return;
    velocity_ = value; velocity_time_ = steady(); velocity_stamp_ = msg->header.stamp;
  }
  void imu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& w = msg->angular_velocity;
    Eigen::Vector3d v(w.x,w.y,w.z);
    if (!v.allFinite() || !validStamp(msg->header.stamp, imu_stamp_)) return;
    angular_rate_ = v.norm(); imu_stamp_ = msg->header.stamp; imu_time_ = steady();
  }
  double pulseHold() const {
    return group_ < plan_.size() && plan_[group_].hold_s > 0 ? plan_[group_].hold_s : pulse_hold_;
  }
  void thrust(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (offboard_ || !position_armed_ || !stateFresh() || msg->header.stamp <= position_since_ ||
        msg->header.stamp <= thrust_stamp_ || age < -clock_lead_tolerance_ || age > .5 ||
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) ||
        !std::isfinite(msg->thrust) || msg->thrust <= 0 || msg->thrust > 1) return;
    const bool was_ready = ready();
    hover_ = msg->thrust; thrust_time_ = steady(); thrust_stamp_ = msg->header.stamp;
    if (!was_ready && ready()) ROS_INFO("Pre-Offboard thrust ready: %.6f; establish stable Position hover before switching", hover_);
  }
  void state(const mavros_msgs::State::ConstPtr& msg) {
    const bool next = msg->connected && msg->armed && msg->mode == "OFFBOARD";
    const bool entering = next && !offboard_;
    const bool leaving = !next && offboard_;
    const bool position = msg->connected && msg->armed && msg->mode == "POSCTL";
    if (entering) {
      // HEARTBEAT reports mode at 1 Hz; body-rate Offboard stops updating
      // ATTITUDE_TARGET immediately. Permit only this bounded handover delay.
      locked_ = ready(1.5);
      if (locked_) {
        origin_ = position_; origin_yaw_ = yaw_; altitude_.integral = 0; control_stamp_ = ros::Time::now();
      }
    }
    if (!position || !position_armed_ || !stateFresh()) {
      thrust_time_ = 0; thrust_stamp_ = ros::Time(); position_since_ = ros::Time::now();
    }
    position_armed_ = position; offboard_ = next; state_time_ = steady();
    if (leaving) {
      active_ = pulsing_ = false;
      const bool finished = exiting_ && complete_ && msg->connected && msg->mode == exit_mode_;
      phase_ = finished ? "complete" : "aborted";
      if (!finished && reason_.empty()) reason_ = "manual exit or FCU disconnected";
      exiting_ = false; complete_ = finished; locked_ = false;
      publishFinished(finished); publishStatus();
    }
    if (entering) {
      complete_ = false; reason_.clear();
      if (!locked_) requestExit("pre-entry thrust or motion feedback unavailable");
      else {
        ROS_INFO("Offboard entered: hover thrust locked at %.6f; %zu pulse groups", hover_, plan_.size());
        begin();
      }
    }
    if (!next) locked_ = false;
  }
  bool centered() const {
    const double yaw_error = std::atan2(std::sin(yaw_-origin_yaw_),std::cos(yaw_-origin_yaw_));
    return motionFresh() && angular_rate_ <= max_body_rate_ && (origin_-position_).head<2>().norm() <= horizontal_tolerance_ &&
      std::abs(origin_.z()-position_.z()) <= height_tolerance_ &&
      velocity_.norm() <= speed_tolerance_ && cos_tilt_ >= std::cos(tilt_tolerance_*M_PI/180) &&
      std::abs(yaw_error) <= yaw_tolerance_*M_PI/180;
  }
  void begin() {
    group_ = pulse_ = 0; active_ = true; pulsing_ = exiting_ = complete_ = false;
    stage_time_ = steady(); stable_time_ = 0; phase_ = "recover"; reason_.clear();
    publishFinished(false); publishStatus();
  }
  void requestExit(const std::string& reason) {
    active_ = pulsing_ = complete_ = false; exiting_ = offboard_;
    phase_ = "aborted"; reason_ = reason;
    publishFinished(false); publishStatus();
    ROS_WARN("Identification stopped: %s; requesting %s", reason.c_str(), exit_mode_.c_str());
  }
  void protection() {
    if (!offboard_ || exiting_) return;
    if (!stateFresh()) requestExit("FCU state stale");
    else if (!locked_ || !motionFresh()) requestExit("motion feedback unavailable or stale");
    else if (cos_tilt_ < std::cos(max_tilt_*M_PI/180)) requestExit("tilt limit exceeded");
    else if (std::abs(origin_.z()-position_.z()) > max_height_) requestExit("height deviation limit exceeded");
    else if ((position_-origin_).norm() > max_distance_) requestExit("distance limit exceeded");
    else if (std::abs(velocity_.z()) > max_vz_) requestExit("vertical speed limit exceeded");
  }
  void advance() {
    if (!active_) return;
    const double now = steady();
    if (pulsing_) {
      const double elapsed = plan_[group_].sine_hz>0 ? (ros::Time::now()-wave_start_).toSec() : now-stage_time_;
      if(elapsed<0) { requestExit("excitation clock moved backwards"); return; }
      if (elapsed < pulseHold()) return;
      if (elapsed > pulseHold()+.1) { requestExit("pulse scheduler overrun"); return; }
      ++pulse_;
      if (pulse_ == plan_[group_].pulses.size()) {
        pulsing_ = false; ++group_; pulse_ = 0; stable_time_ = 0; phase_ = "recover";
      }
      stage_time_ = now; publishStatus(); return;
    }
    if (now-stage_time_ > recovery_timeout_) { requestExit("recenter timeout"); return; }
    if (!centered()) { stable_time_ = 0; return; }
    if (stable_time_ == 0) stable_time_ = now;
    if (now-stable_time_ < settle_) return;
    if (group_ == plan_.size()) {
      active_ = false; complete_ = true; exiting_ = true; phase_ = "waiting_exit";
    } else {
      // 推力辨识双脉冲期间冻结闭环基准，不让高度反馈抵消待辨识的推力增量。
      thrust_base_ = sent_thrust_;
      if (plan_[group_].channel == "thrust") {
        for (const auto& pulse : plan_[group_].pulses)
          if (thrust_base_+pulse[3] <= altitude_.min_thrust || thrust_base_+pulse[3] >= altitude_.max_thrust) {
            requestExit("thrust pulse would saturate"); return;
          }
      }
      pulsing_ = true; pulse_ = 0; stage_time_ = now; wave_start_=ros::Time::now(); phase_ = "pulse";
    }
    publishStatus();
  }
  void publishCommand() {
    if (!stateFresh() || (offboard_ ? (!locked_ || !motionFresh()) : !ready(1.5))) return;
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now(); msg.header.frame_id = "base_link";
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; msg.orientation.w = 1;
    msg.thrust = hover_;
    if (offboard_) {
      const double dt = std::max(0.0, (msg.header.stamp-control_stamp_).toSec());
      control_stamp_ = msg.header.stamp;
      if (pulsing_ && plan_[group_].channel == "thrust") {
        // 只在该短双脉冲内冻结高度积分和基准；回中立即恢复闭环。
        msg.thrust = thrust_base_+plan_[group_].pulses[pulse_][3];
      } else {
        msg.thrust = altitude_.update(hover_, origin_.z()-position_.z(), velocity_.z(),
          std::max(cos_tilt_, std::cos(max_tilt_*M_PI/180)), dt);
      }
      Eigen::Vector3d rates;
      if (pulsing_) {
        const auto& p = plan_[group_].pulses[pulse_]; rates = Eigen::Vector3d(p[0],p[1],p[2])*identification::waveformScale(plan_[group_], (msg.header.stamp-wave_start_).toSec());
        if(plan_[group_].sine_hz>0 && plan_[group_].channel=="yaw") {
          // Long yaw-only segments must still brake XY drift and keep roll/pitch level.
          const auto hold=recenter_.rates(attitude_,origin_-position_,velocity_,origin_yaw_);
          rates.x()=hold.x(); rates.y()=hold.y();
        }
      } else rates = recenter_.rates(attitude_, origin_-position_, velocity_, origin_yaw_);
      msg.body_rate.x = rates.x(); msg.body_rate.y = rates.y(); msg.body_rate.z = rates.z();
    }
    sent_thrust_ = msg.thrust; command_pub_.publish(msg);
  }
  void retryExit() {
    if (mode_request_.valid()) {
      if (mode_request_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
      if (!mode_request_.get()) ROS_WARN("Exit mode request failed; retrying");
    }
    if (!exiting_ || !offboard_ || steady()-last_request_ < .5) return;
    last_request_ = steady();
    mode_request_ = std::async(std::launch::async, [this]() {
      auto client = nh_.serviceClient<mavros_msgs::SetMode>(mavros_+"/set_mode");
      mavros_msgs::SetMode request; request.request.custom_mode = exit_mode_;
      return client.call(request) && request.response.mode_sent;
    });
  }
  void publishFinished(bool finished) {
    std_msgs::Bool m; m.data = finished; finished_pub_.publish(m);
  }
  std::string status() const {
    std::ostringstream out;
    out << "{\"state\":\"" << (active_ ? "EXCITING" : exiting_ ? "WAIT_EXIT" : complete_ ? "COMPLETE" : "IDLE")
        << "\",\"phase\":\"" << phase_ << "\",\"channel\":\"" << (group_ < plan_.size() ? plan_[group_].channel : "none")
        << "\",\"segment\":" << group_ << ",\"pulse\":" << pulse_
        << ",\"sine_hz\":" << (group_<plan_.size() ? plan_[group_].sine_hz : 0)
        << ",\"pulse_hold_s\":" << pulseHold() << ",\"body_rate_rad_s\":" << angular_rate_
        << ",\"sequence_complete\":" << (complete_ && !exiting_ ? "true" : "false")
        << ",\"thrust_ready\":" << (ready() ? "true" : "false")
        << ",\"thrust_locked\":" << (locked_ ? "true" : "false")
        << ",\"hover_thrust\":" << hover_ << ",\"height_error_m\":" << origin_.z()-position_.z()
        << ",\"reason\":\"" << reason_ << "\"}";
    return out.str();
  }
  void publishStatus() {
    std_msgs::String m; m.data = status(); status_pub_.publish(m); last_status_ = steady();
  }
  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    r.success = offboard_ && stateFresh() && locked_ && motionFresh() && !active_ && !exiting_ && !complete_;
    r.message = r.success ? "started" : "requires fresh captured Offboard entry and idle non-completed sequence";
    if (r.success) begin();
    return true;
  }
  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    requestExit("operator stop"); r.success = true; r.message = "stopped; requesting exit mode"; return true;
  }
  bool getStatus(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    r.success = true; r.message = status(); return true;
  }
  void tick(const ros::WallTimerEvent&) {
    protection(); advance(); publishCommand(); retryExit();
    if (steady()-last_status_ > .1) publishStatus();
  }
 public:
  Identification() {
    if (!ros::Time::waitForValid(ros::WallDuration(30)))
      throw std::runtime_error("No valid ROS clock after 30 s; start Gazebo /clock before the experiment");
    config();
    command_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(mavros_+"/setpoint_raw/attitude", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/identification/status", 10, true);
    finished_pub_ = priv_.advertise<std_msgs::Bool>("finished", 1, true);
    state_sub_ = nh_.subscribe(mavros_+"/state", 10, &Identification::state, this);
    thrust_sub_ = nh_.subscribe(mavros_+"/setpoint_raw/target_attitude", 1, &Identification::thrust, this);
    pose_sub_ = nh_.subscribe(mavros_+"/local_position/pose", 1, &Identification::pose, this);
    imu_sub_ = nh_.subscribe(mavros_+"/imu/data", 1, &Identification::imu, this);
    velocity_sub_ = nh_.subscribe(mavros_+"/local_position/velocity_local", 1, &Identification::velocity, this);
    services_.push_back(priv_.advertiseService("start", &Identification::start, this));
    services_.push_back(priv_.advertiseService("stop", &Identification::stop, this));
    services_.push_back(priv_.advertiseService("status", &Identification::getStatus, this));
    timer_ = nh_.createWallTimer(ros::WallDuration(1/frequency_), &Identification::tick, this);
    publishFinished(false); publishStatus();
  }
  ~Identification() { if (mode_request_.valid()) mode_request_.wait(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "identification_fsm");
  try { Identification node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
