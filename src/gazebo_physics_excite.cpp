#include <ros/ros.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class GazeboPhysicsExcite {
 public:
  GazeboPhysicsExcite() : nh_(), private_nh_("~") {
    private_nh_.param("frequency", frequency_, 50.0);
    private_nh_.param("hover_thrust", hover_thrust_, 0.24);
    private_nh_.param("thrust_amplitude", thrust_amplitude_, 0.025);
    private_nh_.param("pulse_hold", pulse_hold_, 0.5);
    private_nh_.param("baseline_seconds", baseline_seconds_, 0.5);
    private_nh_.param("recovery_seconds", recovery_seconds_, 2.0);
    private_nh_.param("repetitions", repetitions_, 2);
    private_nh_.param<std::vector<double>>("rate_amplitudes", rate_amplitudes_,
                                           std::vector<double>{0.30, 0.30, 0.25});
    validateParameters();

    command_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/identification/status", 1, true);
    finished_pub_ = private_nh_.advertise<std_msgs::Bool>("finished", 1, true);
    start_service_ = private_nh_.advertiseService("start", &GazeboPhysicsExcite::start, this);
    stop_service_ = private_nh_.advertiseService("stop", &GazeboPhysicsExcite::stop, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &GazeboPhysicsExcite::tick, this);
    publishFinished(false);
  }

 private:
  static constexpr int kExperimentCount = 5;

  void validateParameters() {
    const bool rates_ok = rate_amplitudes_.size() == 3 &&
        std::all_of(rate_amplitudes_.begin(), rate_amplitudes_.end(),
                    [](double value) { return std::isfinite(value) && value > 0.0; });
    if (!std::isfinite(frequency_) || frequency_ <= 0.0 ||
        !std::isfinite(hover_thrust_) || !std::isfinite(thrust_amplitude_) ||
        hover_thrust_ - thrust_amplitude_ <= 0.0 ||
        hover_thrust_ + thrust_amplitude_ >= 1.0 || pulse_hold_ <= 0.0 ||
        baseline_seconds_ <= 0.0 || recovery_seconds_ <= 0.0 ||
        repetitions_ < 1 || !rates_ok) {
      throw std::runtime_error("invalid Gazebo physics excitation parameters");
    }
  }

  bool start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (active_) {
      response.success = false;
      response.message = "excitation already active";
      return true;
    }
    active_ = true;
    recovering_ = false;
    plan_index_ = 0;
    phase_start_ = ros::Time::now();
    publishFinished(false);
    response.success = true;
    response.message = "Gazebo physics excitation started";
    return true;
  }

  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    active_ = false;
    recovering_ = false;
    response.success = true;
    response.message = "Gazebo physics excitation stopped";
    publishStatus("aborted");
    return true;
  }

  std::string currentExperiment() const {
    static const std::array<std::string, kExperimentCount> names{{
        "thrust", "roll", "pitch", "yaw", "coupled"}};
    return names[std::min(plan_index_ % kExperimentCount, kExperimentCount - 1)];
  }

  void publishFinished(bool finished) {
    std_msgs::Bool message;
    message.data = finished;
    finished_pub_.publish(message);
  }

  void publishStatus(const std::string& phase) {
    std_msgs::String message;
    std::ostringstream json;
    json << "{\"state\":\"" << (active_ ? "EXCITING" : "IDLE")
         << "\",\"phase\":\"" << phase << "\",\"channel\":\""
         << currentExperiment() << "\",\"segment\":" << plan_index_
         << ",\"sequence_complete\":"
         << (!active_ && plan_index_ >= repetitions_ * kExperimentCount ? "true" : "false")
         << "}";
    message.data = json.str();
    status_pub_.publish(message);
  }

  void publishCommand(const std::array<double, 4>& action, const ros::Time& stamp) {
    mavros_msgs::AttitudeTarget command;
    command.header.stamp = stamp;
    command.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    command.body_rate.x = action[0];
    command.body_rate.y = action[1];
    command.body_rate.z = action[2];
    command.thrust = action[3];
    command_pub_.publish(command);
  }

  std::array<double, 4> excitation(int pulse) const {
    std::array<double, 4> action{{0.0, 0.0, 0.0, hover_thrust_}};
    const int experiment = plan_index_ % kExperimentCount;
    const double sign = pulse % 2 == 0 ? 1.0 : -1.0;
    if (experiment == 0) {
      action[3] += sign * thrust_amplitude_;
    } else if (experiment < 4) {
      action[experiment - 1] = sign * rate_amplitudes_[experiment - 1];
    } else {
      static const std::array<std::array<double, 3>, 4> signs{{
          {{1.0, 1.0, 1.0}}, {{1.0, -1.0, -1.0}},
          {{-1.0, 1.0, -1.0}}, {{-1.0, -1.0, 1.0}}}};
      for (int axis = 0; axis < 3; ++axis) {
        action[axis] = signs[pulse % 4][axis] * rate_amplitudes_[axis];
      }
    }
    return action;
  }

  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const std::array<double, 4> hover{{0.0, 0.0, 0.0, hover_thrust_}};
    if (!active_) {
      publishCommand(hover, now);
      publishStatus("idle");
      return;
    }
    if (recovering_) {
      publishCommand(hover, now);
      publishStatus("recover");
      if ((now - phase_start_).toSec() >= recovery_seconds_) {
        ++plan_index_;
        if (plan_index_ >= repetitions_ * kExperimentCount) {
          active_ = false;
          recovering_ = false;
          publishStatus("complete");
          publishFinished(true);
          ROS_INFO("Gazebo physics excitation sequence completed");
        } else {
          phase_start_ = now;
          recovering_ = false;
        }
      }
      return;
    }

    const double elapsed = (now - phase_start_).toSec();
    const double pulse_duration = 4.0 * pulse_hold_;
    const double cycle_end = 2.0 * baseline_seconds_ + pulse_duration;
    std::array<double, 4> action = hover;
    if (elapsed >= baseline_seconds_ && elapsed < baseline_seconds_ + pulse_duration) {
      const int pulse = std::min(3, static_cast<int>((elapsed - baseline_seconds_) / pulse_hold_));
      action = excitation(pulse);
    }
    publishCommand(action, now);
    publishStatus("pulse");
    if (elapsed >= cycle_end) {
      recovering_ = true;
      phase_start_ = now;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::Publisher finished_pub_;
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  ros::Timer timer_;
  std::vector<double> rate_amplitudes_;
  double frequency_{50.0};
  double hover_thrust_{0.24};
  double thrust_amplitude_{0.025};
  double pulse_hold_{0.5};
  double baseline_seconds_{0.5};
  double recovery_seconds_{2.0};
  int repetitions_{2};
  int plan_index_{0};
  bool active_{false};
  bool recovering_{false};
  ros::Time phase_start_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_physics_excite");
  try {
    GazeboPhysicsExcite node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
  return 0;
}
