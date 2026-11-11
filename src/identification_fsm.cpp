/**
 * @brief 统一调度两类辨识波形：Offboard 位置回中 → 推力采样 → 零角速度交接 → 脉冲组 → 回中。
 * 激励时采用 FLU 角速度和高度闭环；回中由 PX4 位置控制器完成。
 * 不解锁、不起飞；完成或保护触发后保持 Offboard 位置，人工退出立即停止激励。
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <holydrone_identification/recenter_control.hpp>
#include <holydrone_identification/excitation_plan.hpp>
#include <chrono>
#include <deque>
#include <holydrone_identification/settling_gate.hpp>
#include <sstream>
#include <stdexcept>

static double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class Identification {
  ros::NodeHandle nh_, priv_{"~"};
  ros::Publisher command_pub_, position_pub_, status_pub_, finished_pub_;
  ros::Subscriber state_sub_, thrust_sub_, pose_sub_, velocity_sub_, imu_sub_;
  ros::WallTimer timer_;
  std::vector<ros::ServiceServer> services_;
  std::string mavros_, exciter_, phase_ = "waiting", reason_;
  mavros_msgs::State fcu_state_;
  std::string last_gate_;
  uint64_t command_count_ = 0, rate_count_ = 0;
  double rate_since_ = 0, command_hz_ = 0, last_gate_log_ = 0;
  // Diagnostic-only counters: do not weaken freshness or arming requirements.
  uint64_t pose_rejected_ = 0, velocity_rejected_ = 0, imu_rejected_ = 0;
  std::vector<identification::Group> plan_;
  size_t group_ = 0, pulse_ = 0;
  bool offboard_ = false, hover_armed_ = false, locked_ = false, thrust_locked_ = false;
  bool active_ = false, pulsing_ = false, complete_ = false;
  double hover_ = 0, thrust_base_ = 0;
  double state_time_ = 0, thrust_time_ = 0, pose_time_ = 0, velocity_time_ = 0;
  double stage_time_ = 0, last_status_ = 0;
  ros::Time wave_start_;
  ros::Time position_since_, thrust_stamp_, pose_stamp_, velocity_stamp_, control_stamp_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero(), origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  double yaw_ = 0, origin_yaw_ = 0, cos_tilt_ = 1;
  double clock_lead_tolerance_ = 0;
  double angular_rate_ = 0, settled_angular_rate_ = 0, angular_filter_tau_ = .05;
  double imu_time_ = 0, angular_timeout_ = .1;
  ros::Time imu_stamp_;
  double frequency_ = 200, pulse_hold_ = .35, settle_ = .3;
  double height_tolerance_ = .15, horizontal_tolerance_ = .75;
  double yaw_tolerance_ = 5, feedback_timeout_ = .5;
  double horizontal_speed_tolerance_ = .3, tilt_tolerance_ = 5, angular_speed_tolerance_ = 5;
  identification::SettlingGate settling_;
  identification::AngularRateFilter angular_filter_;
  std::deque<std::pair<double, double>> hover_samples_;
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
    const bool physics = exciter_ == "gazebo_physics_excite";
    const bool second = exciter_ == "brct2order_excite";
    const bool third = exciter_ == "brct3order_excite";
    if (!physics && !second && !third && exciter_ != "brct1order_excite") throw std::runtime_error("unknown exciter");
    pulse_hold_ = physics ? .5 : .35;
    std::vector<double> rates = physics ? std::vector<double>{.3,.3,.25} : std::vector<double>{.2,.2,.25};
    priv_.param("rate_amplitudes", rates, rates);
    double thrust = physics ? .025 : .015;
    int repetitions = physics ? 2 : 3;
    priv_.param("repetitions", repetitions, repetitions);
    positive("frequency", frequency_); positive("pulse_hold", pulse_hold_); positive("thrust_amplitude", thrust);
    positive("recovery/settle_s", settle_);
    positive("recovery/height_tolerance_m", height_tolerance_);
    positive("recovery/horizontal_tolerance_m", horizontal_tolerance_);
    positive("recovery/yaw_tolerance_deg", yaw_tolerance_);
    positive("recovery/horizontal_speed_tolerance_m_s", horizontal_speed_tolerance_);
    positive("recovery/tilt_tolerance_deg", tilt_tolerance_);
    positive("recovery/angular_speed_tolerance_deg_s", angular_speed_tolerance_);
    positive("recovery/angular_rate_filter_tau_s", angular_filter_tau_);
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
    positive("protection/angular_feedback_timeout_s", angular_timeout_);
    if (frequency_ < 20 || frequency_ > 500 || repetitions < 1 || rates.size() != 3 ||
        angular_filter_tau_ > settle_ ||
        recenter_.max_tilt_deg >= 80 || yaw_tolerance_ >= 180 || tilt_tolerance_ >= 90 ||
        altitude_.min_thrust >= altitude_.max_thrust || altitude_.max_thrust > 1 || thrust >= .5)
      throw std::runtime_error("inconsistent excitation/control/protection limits");
    for (double r : rates) if (!std::isfinite(r) || r <= 0) throw std::runtime_error("invalid rate amplitude");
    const std::array<double, 3> a{{rates[0],rates[1],rates[2]}};
    std::vector<double> durations;
    if (second || third) durations = {.2, .35, .5};
    if (physics || second || third) priv_.getParam("pulse_durations_s", durations);
    for (double d : durations) if (!std::isfinite(d) || d < .1 || d > .75)
      throw std::runtime_error("pulse_durations_s must be within [0.1, 0.75] s");
    plan_ = physics ? identification::physicsPlan(a, thrust, repetitions, durations) :
      third ? identification::thirdOrderPlan(a, thrust, repetitions, durations) :
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
      if (priv_.hasParam(old)) ROS_WARN("%s is no longer used; thrust is captured in stable OFFBOARD position control, recovery is feedback-gated", old);
  }
  bool motionFresh() const {
    return pose_time_ > 0 && velocity_time_ > 0 && imu_time_ > 0 &&
      identification::freshSample((ros::Time::now()-imu_stamp_).toSec(), steady()-imu_time_,
        angular_timeout_, feedback_timeout_, clock_lead_tolerance_) &&
      steady()-pose_time_ < feedback_timeout_ && steady()-velocity_time_ < feedback_timeout_;
  }
  bool stateFresh() const { return state_time_ > 0 && steady()-state_time_ < 2; }
  bool ready() const {
    return offboard_ && stateFresh() && motionFresh() &&
      (ros::Time::now()-thrust_stamp_).toSec() >= -clock_lead_tolerance_ &&
      (ros::Time::now()-thrust_stamp_).toSec() < .5 && thrust_time_ > 0 && steady()-thrust_time_ < .5 &&
      hover_samples_.size() >= 3 && hover_samples_.back().first-hover_samples_.front().first >= .2;
  }
  void resetRecovery() {
    position_since_ = ros::Time::now(); thrust_time_ = 0; thrust_locked_ = false;
    hover_samples_.clear(); settling_.reset();
  }
  bool validStamp(const ros::Time& stamp, const ros::Time& previous) const {
    const double age = (ros::Time::now()-stamp).toSec();
    return stamp > previous && age >= -clock_lead_tolerance_ && age <= feedback_timeout_;
  }
  void pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const auto& p = msg->pose.position; const auto& q = msg->pose.orientation;
    Eigen::Vector3d v(p.x,p.y,p.z); Eigen::Quaterniond a(q.w,q.x,q.y,q.z);
    if (!v.allFinite() || !a.coeffs().allFinite() || a.squaredNorm() < .5 || a.squaredNorm() > 1.5 ||
        !validStamp(msg->header.stamp, pose_stamp_)) { ++pose_rejected_; return; }
    position_ = v; attitude_ = a.normalized();
    cos_tilt_ = identification::bound(1-2*(attitude_.x()*attitude_.x()+attitude_.y()*attitude_.y()), -1, 1);
    yaw_ = std::atan2(2*(attitude_.w()*attitude_.z()+attitude_.x()*attitude_.y()),
                     1-2*(attitude_.y()*attitude_.y()+attitude_.z()*attitude_.z()));
    pose_time_ = steady(); pose_stamp_ = msg->header.stamp;
  }
  void velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const auto& v = msg->twist.linear; Eigen::Vector3d value(v.x,v.y,v.z);
    if (!value.allFinite() || !validStamp(msg->header.stamp, velocity_stamp_)) { ++velocity_rejected_; return; }
    velocity_ = value; velocity_time_ = steady(); velocity_stamp_ = msg->header.stamp;
  }
  void imu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& w = msg->angular_velocity;
    Eigen::Vector3d v(w.x,w.y,w.z);
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (!v.allFinite() || msg->angular_velocity_covariance[0] == -1 ||
        !validStamp(msg->header.stamp, imu_stamp_) || age > angular_timeout_) {
      ++imu_rejected_; settling_.reset(); angular_filter_.reset(); return;
    }
    angular_rate_ = v.norm(); imu_stamp_ = msg->header.stamp; imu_time_ = steady();
    settled_angular_rate_ = angular_filter_.observe(imu_stamp_.toSec(), v, angular_filter_tau_, angular_timeout_);
    settling_.observe(imu_stamp_.toSec(), active_ && !pulsing_ && centered(), angular_timeout_);
  }
  double pulseHold() const {
    return group_ < plan_.size() && plan_[group_].hold_s > 0 ? plan_[group_].hold_s : pulse_hold_;
  }
  void thrust(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!offboard_ || !locked_ || pulsing_) return;
    if (!centered()) { hover_samples_.clear(); thrust_time_ = 0; return; }
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if ((msg->header.stamp-position_since_).toSec() < .65 || msg->header.stamp <= thrust_stamp_ ||
        age < -clock_lead_tolerance_ || age > .5 ||
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) ||
        !std::isfinite(msg->thrust) || msg->thrust <= 0 || msg->thrust > 1) return;
    const double stamp = msg->header.stamp.toSec();
    if (!hover_samples_.empty() && stamp-hover_samples_.back().first > .2) hover_samples_.clear();
    hover_samples_.emplace_back(stamp, msg->thrust);
    while (!hover_samples_.empty() && stamp-hover_samples_.front().first > .5) hover_samples_.pop_front();
    thrust_stamp_ = msg->header.stamp; thrust_time_ = steady();
  }
  void state(const mavros_msgs::State::ConstPtr& msg) {
    fcu_state_ = *msg;
    const bool next = msg->connected && msg->armed && msg->mode == "OFFBOARD";
    const bool entering = next && !offboard_, leaving = !next && offboard_;
    hover_armed_ = msg->connected && msg->armed && (msg->mode == "POSCTL" || msg->mode == "AUTO.LOITER");
    offboard_ = next; state_time_ = steady();
    if (leaving) {
      active_ = pulsing_ = locked_ = false;
      if (!complete_) { phase_ = "aborted"; reason_ = "manual exit or FCU disconnected"; }
      resetRecovery(); publishStatus();
    }
    if (entering) {
      complete_ = false; reason_.clear(); locked_ = motionFresh();
      if (!locked_) abortSequence("motion feedback unavailable at entry");
      else {
        origin_ = position_; origin_yaw_ = yaw_;
        ROS_INFO("Offboard entered: position origin captured; %zu pulse groups", plan_.size());
        begin();
      }
    }
  }
  std::string recoveryGate() const {
    const double yaw_error = std::atan2(std::sin(yaw_-origin_yaw_),std::cos(yaw_-origin_yaw_));
    if (!motionFresh()) return "motion feedback unavailable or stale";
    if (std::hypot(velocity_.x(), velocity_.y()) > horizontal_speed_tolerance_) return "horizontal speed";
    if (cos_tilt_ < std::cos(tilt_tolerance_*M_PI/180)) return "tilt";
    if (settled_angular_rate_ > angular_speed_tolerance_*M_PI/180) return "filtered angular speed";
    if ((origin_-position_).head<2>().norm() > horizontal_tolerance_) return "horizontal position";
    if (std::abs(origin_.z()-position_.z()) > height_tolerance_) return "height error";
    if (std::abs(yaw_error) > yaw_tolerance_*M_PI/180) return "yaw error";
    return "";
  }
  bool centered() const { return recoveryGate().empty(); }
  std::string excitationGate() const {
    if (!active_) return reason_.empty() ? phase_ : reason_;
    if (pulsing_) return "pulse running";
    const auto gate = recoveryGate();
    if (!gate.empty()) return gate;
    if (!settling_.ready(settle_)) return "settling window";
    if (group_ == plan_.size()) return "ready";
    if ((ros::Time::now()-position_since_).toSec() < .65) return "position thrust isolation";
    if (hover_samples_.empty()) return "waiting for valid FCU thrust";
    if (hover_samples_.size() < 3 || hover_samples_.back().first-hover_samples_.front().first < .2)
      return "thrust sample window";
    if (!ready()) return "thrust or FCU feedback stale";
    return "ready";
  }
  void begin() {
    group_ = pulse_ = 0; active_ = true; pulsing_ = complete_ = false;
    stage_time_ = steady(); resetRecovery(); phase_ = "recover"; reason_.clear();
    ROS_INFO("Identification: initial recenter; %zu groups", plan_.size());
    publishFinished(false); publishStatus();
  }
  void abortSequence(const std::string& reason) {
    active_ = pulsing_ = complete_ = false;
    resetRecovery();
    phase_ = "aborted"; reason_ = reason; settling_.reset();
    publishFinished(false); publishStatus();
    ROS_WARN("Identification stopped: %s; holding PX4 position in OFFBOARD", reason.c_str());
  }
  void protection() {
    if (!offboard_ || !active_) return;
    if (!stateFresh()) abortSequence("FCU state stale");
    else if (!locked_ || !motionFresh()) abortSequence("motion feedback unavailable or stale");
  }
  void startPulse() {
    // 推力辨识保留冻结基准，避免高度闭环抵消待辨识的推力增量。
    thrust_base_ = hover_;
    if (plan_[group_].channel == "thrust") {
      for (const auto& pulse : plan_[group_].pulses)
        if (thrust_base_+pulse[3] <= altitude_.min_thrust || thrust_base_+pulse[3] >= altitude_.max_thrust) {
          abortSequence("thrust pulse would saturate"); return;
        }
    }
    pulsing_ = true;
    stage_time_ = steady(); wave_start_ = ros::Time::now(); phase_ = "pulse"; publishStatus();
    const auto& g = plan_[group_];
    const auto& p = g.pulses[pulse_];
    ROS_INFO("Identification [%zu/%zu] %s pulse %zu/%zu: rates=(%+.3f,%+.3f,%+.3f) rad/s, thrust_delta=%+.3f, duration=%.2f s",
      group_+1, plan_.size(), g.channel.c_str(), pulse_+1, g.pulses.size(),
      p[0], p[1], p[2], p[3], pulseHold());
    if (g.sine_hz > 0) ROS_INFO("Identification: sine excitation %.2f Hz (rates above are amplitudes)", g.sine_hz);
  }
  void advance() {
    if (!active_) return;
    const double now = steady();
    if (pulsing_) {
      const double elapsed = plan_[group_].sine_hz>0 ? (ros::Time::now()-wave_start_).toSec() : now-stage_time_;
      if (elapsed<0) { abortSequence("excitation clock moved backwards"); return; }
      if (elapsed < pulseHold()) return;
      if (elapsed > pulseHold()+.1) { abortSequence("pulse scheduler overrun"); return; }
      ROS_INFO("Identification [%zu/%zu] %s pulse %zu/%zu ended; recentering",
        group_+1, plan_.size(), plan_[group_].channel.c_str(), pulse_+1, plan_[group_].pulses.size());
      ++pulse_;
      if (pulse_ == plan_[group_].pulses.size()) { ++group_; pulse_ = 0; }
      pulsing_ = false; settling_.reset(); phase_ = "recover";
      resetRecovery();
      stage_time_ = now; publishStatus(); return;
    }
    if (!centered()) { settling_.reset(); return; }
    if (!settling_.ready(settle_)) return;
    if (group_ == plan_.size()) {
      active_ = false; complete_ = true; phase_ = "complete"; publishFinished(true);
      ROS_INFO("Identification complete: all %zu groups finished and recentered; holding PX4 position in OFFBOARD", plan_.size());
    } else {
      if (!ready()) return;
      hover_ = 0;
      for (const auto& sample : hover_samples_) hover_ += sample.second;
      hover_ /= hover_samples_.size(); thrust_locked_ = true; altitude_.integral = 0; control_stamp_ = ros::Time::now();
      startPulse();
    }
    publishStatus();
  }
  std::string commandGate() const {
    if (!stateFresh()) return "FCU state missing or stale";
    if (!fcu_state_.connected) return "FCU disconnected";
    if (!fcu_state_.armed) return "waiting for manual arming and Position hover";
    if (!offboard_ && !hover_armed_) return "requires POSCTL or AUTO.LOITER before OFFBOARD";
    if (pose_time_ <= 0) return "no accepted position feedback";
    if (velocity_time_ <= 0) return "no accepted velocity feedback";
    if (imu_time_ <= 0) return "no accepted angular feedback";
    if (steady()-pose_time_ >= feedback_timeout_) return "position feedback stale";
    if (steady()-velocity_time_ >= feedback_timeout_) return "velocity feedback stale";
    if (!motionFresh()) return "IMU timestamp or receipt freshness failed";
    if (offboard_ && !locked_) return "OFFBOARD origin unavailable";
    if (pulsing_ && !thrust_locked_) return "rate command requires captured thrust";
    return "streaming";
  }
  void publishCommand() {
    if (commandGate() != "streaming") return;
    if (!pulsing_) {
      mavros_msgs::PositionTarget msg;
      msg.header.stamp = ros::Time::now(); msg.header.frame_id = "map";
      msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
      msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
      const auto& target = offboard_ ? origin_ : position_;
      msg.position.x = target.x(); msg.position.y = target.y(); msg.position.z = target.z();
      msg.yaw = offboard_ ? origin_yaw_ : yaw_; position_pub_.publish(msg); ++command_count_; return;
    }
    if (!thrust_locked_) return;
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
          std::max(cos_tilt_, std::sqrt(.5)), dt);
      }
      Eigen::Vector3d rates;
      if (pulsing_) {
        const auto& p = plan_[group_].pulses[pulse_]; rates = Eigen::Vector3d(p[0],p[1],p[2])*identification::waveformScale(plan_[group_], (msg.header.stamp-wave_start_).toSec());
        if(plan_[group_].sine_hz>0 && plan_[group_].channel=="yaw") {
          // Long yaw-only segments must still brake XY drift and keep roll/pitch level.
          const auto hold=recenter_.rates(attitude_,origin_-position_,velocity_,origin_yaw_);
          rates.x()=hold.x(); rates.y()=hold.y();
        }
      } else rates.setZero();  // 未激励时角速度置零；实际回中发布 PX4 位置设定值。
      msg.body_rate.x = rates.x(); msg.body_rate.y = rates.y(); msg.body_rate.z = rates.z();
    }
    command_pub_.publish(msg); ++command_count_;
  }
  void publishFinished(bool finished) {
    std_msgs::Bool m; m.data = finished; finished_pub_.publish(m);
  }
  std::string status() const {
    std::ostringstream out;
    out << "{\"state\":\"" << (active_ ? "EXCITING" : complete_ ? "COMPLETE" : "IDLE")
        << "\",\"phase\":\"" << phase_ << "\",\"channel\":\"" << (group_ < plan_.size() ? plan_[group_].channel : "none")
        << "\",\"segment\":" << group_ << ",\"pulse\":" << pulse_
        << ",\"sine_hz\":" << (group_<plan_.size() ? plan_[group_].sine_hz : 0)
        << ",\"pulse_hold_s\":" << pulseHold() << ",\"body_rate_rad_s\":" << angular_rate_
        << ",\"settled_body_rate_rad_s\":" << settled_angular_rate_
        << ",\"excitation_gate\":\"" << excitationGate() << "\""
        << ",\"settled_s\":" << settling_.elapsed()
        << ",\"thrust_sample_count\":" << hover_samples_.size()
        << ",\"thrust_sample_span_s\":" << (hover_samples_.empty() ? 0 : hover_samples_.back().first-hover_samples_.front().first)
        << ",\"control_mode\":\"" << (pulsing_ ? "body_rate_altitude" : "px4_position") << "\""
        << ",\"sequence_complete\":" << (complete_ ? "true" : "false")
        << ",\"thrust_ready\":" << (ready() ? "true" : "false")
        << ",\"origin_locked\":" << (locked_ ? "true" : "false")
        << ",\"thrust_locked\":" << (thrust_locked_ ? "true" : "false")
        << ",\"hover_thrust\":" << hover_ << ",\"height_error_m\":" << origin_.z()-position_.z()
        << ",\"reason\":\"" << reason_ << "\"}";
    std::string json = out.str();
    json.pop_back();
    std::ostringstream diagnostic;
    diagnostic << ",\"fcu_mode\":\"" << fcu_state_.mode << "\""
      << ",\"fcu_armed\":" << (fcu_state_.armed ? "true" : "false")
      << ",\"command_gate\":\"" << commandGate() << "\""
      << ",\"command_hz\":" << command_hz_ << ",\"command_count\":" << command_count_
      << ",\"position_subscribers\":" << position_pub_.getNumSubscribers()
      << ",\"imu_age_s\":" << (imu_time_ > 0 ? (ros::Time::now()-imu_stamp_).toSec() : -1)
      << ",\"pose_receipt_age_s\":" << (pose_time_ > 0 ? steady()-pose_time_ : -1)
      << ",\"velocity_receipt_age_s\":" << (velocity_time_ > 0 ? steady()-velocity_time_ : -1)
      << ",\"pose_rejected\":" << pose_rejected_
      << ",\"velocity_rejected\":" << velocity_rejected_
      << ",\"imu_rejected\":" << imu_rejected_ << "}";
    return json+diagnostic.str();
  }
  void publishStatus() {
    std_msgs::String m; m.data = status(); status_pub_.publish(m); last_status_ = steady();
  }
  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    r.success = offboard_ && stateFresh() && locked_ && motionFresh() && !active_ && !complete_;
    r.message = r.success ? "started" : "requires fresh captured Offboard entry and idle non-completed sequence";
    if (r.success) begin();
    return true;
  }
  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    abortSequence("operator stop"); r.success = true; r.message = "stopped; holding PX4 position"; return true;
  }
  bool getStatus(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    r.success = true; r.message = status(); return true;
  }
  void tick(const ros::WallTimerEvent&) {
    protection();
    advance(); publishCommand();
    const double now = steady();
    if (rate_since_ == 0) { rate_since_ = now; rate_count_ = command_count_; }
    if (now-rate_since_ >= 1) {
      command_hz_ = (command_count_-rate_count_)/(now-rate_since_);
      rate_since_ = now; rate_count_ = command_count_;
    }
    const std::string gate = commandGate();
    if (gate != last_gate_ && now-last_gate_log_ >= 1) {
      ROS_INFO("Identification setpoint gate: %s; mode=%s armed=%d", gate.c_str(),
               fcu_state_.mode.c_str(), fcu_state_.armed);
      last_gate_ = gate; last_gate_log_ = now;
    }
    if (!offboard_ && hover_armed_)
      ROS_INFO_THROTTLE(5, "Identification pre-OFFBOARD: %s; sent=%.1f Hz subscribers=%u imu_age=%.3f s rejected=%llu",
        gate.c_str(), command_hz_, position_pub_.getNumSubscribers(),
        (ros::Time::now()-imu_stamp_).toSec(), static_cast<unsigned long long>(imu_rejected_));
    if (offboard_ && active_ && !pulsing_)
      ROS_INFO_THROTTLE(2, "Identification waiting: %s; raw_rate=%.2f filtered_rate=%.2f deg/s settled=%.2f/%.2f s thrust_samples=%zu",
        excitationGate().c_str(), angular_rate_*180/M_PI, settled_angular_rate_*180/M_PI,
        settling_.elapsed(), settle_, hover_samples_.size());
    if (steady()-last_status_ > .1) publishStatus();
  }
 public:
  Identification() {
    if (!ros::Time::waitForValid(ros::WallDuration(30)))
      throw std::runtime_error("No valid ROS clock after 30 s; start Gazebo /clock before the experiment");
    config();
    command_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(mavros_+"/setpoint_raw/attitude", 1);
    position_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(mavros_+"/setpoint_raw/local", 1);
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
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "identification_fsm");
  try { Identification node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
