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

class BrctFirstOrderExcite {
 public:
  BrctFirstOrderExcite() : nh_(), private_nh_("~") {
    private_nh_.param("frequency", frequency_, 50.0);
    private_nh_.param("hover_thrust", hover_thrust_, 0.24);
    private_nh_.param("thrust_amplitude", thrust_amplitude_, 0.015);
    private_nh_.param("pulse_hold", pulse_hold_, 0.35);
    private_nh_.param("baseline_seconds", baseline_seconds_, 0.30);
    private_nh_.param("recovery_seconds", recovery_seconds_, 2.0);
    private_nh_.param("repetitions", repetitions_, 3);
    private_nh_.param<std::vector<double>>("rate_amplitudes", rate_amplitudes_,
                                           std::vector<double>{0.20, 0.20, 0.25});
    validateParameters();

    command_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/identification/status", 1, true);
    finished_pub_ = private_nh_.advertise<std_msgs::Bool>("finished", 1, true);
    start_service_ = private_nh_.advertiseService("start", &BrctFirstOrderExcite::start, this);
    stop_service_ = private_nh_.advertiseService("stop", &BrctFirstOrderExcite::stop, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &BrctFirstOrderExcite::tick, this);
    publishFinished(false);
  }

 private:
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
      throw std::runtime_error("invalid BRCT excitation parameters");
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
    response.message = "BRCT excitation started";
    return true;
  }

  bool stop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    active_ = false;
    recovering_ = false;
    response.success = true;
    response.message = "BRCT excitation stopped";
    publishStatus("aborted", currentChannel(), plan_index_);
    return true;
  }

  std::string currentChannel() const {
    static const std::array<std::string, 4> channels{{"roll", "pitch", "yaw", "thrust"}};
    return channels[std::min(plan_index_ % 4, 3)];
  }

  void publishFinished(bool finished) {
    std_msgs::Bool message;
    message.data = finished;
    finished_pub_.publish(message);
  }

  void publishStatus(const std::string& phase, const std::string& channel, int segment) {
    std_msgs::String message;
    std::ostringstream json;
    json << "{\"state\":\"" << (active_ ? "EXCITING" : "IDLE")
         << "\",\"phase\":\"" << phase << "\",\"channel\":\"" << channel
         << "\",\"segment\":" << segment
         << ",\"sequence_complete\":" << (!active_ && plan_index_ >= repetitions_ * 4 ? "true" : "false")
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

  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    std::array<double, 4> action{{0.0, 0.0, 0.0, hover_thrust_}};
    if (!active_) {
      publishCommand(action, now);  // Pre-stream so PX4 permits manual OFFBOARD entry.
      publishStatus("idle", "none", -1);
      return;
    }

    if (recovering_) {
      publishCommand(action, now);
      publishStatus("recover", currentChannel(), plan_index_);
      if ((now - phase_start_).toSec() >= recovery_seconds_) {
        ++plan_index_;
        if (plan_index_ >= repetitions_ * 4) {
          active_ = false;
          recovering_ = false;
          publishStatus("complete", "none", plan_index_);
          publishFinished(true);
          ROS_INFO("BRCT excitation sequence completed");
        } else {
          phase_start_ = now;
          recovering_ = false;
        }
      }
      return;
    }

    const double elapsed = (now - phase_start_).toSec();
    const double excitation_end = baseline_seconds_ + 2.0 * pulse_hold_;
    const double cycle_end = excitation_end + baseline_seconds_;
    const int channel = plan_index_ % 4;
    if (elapsed >= baseline_seconds_ && elapsed < excitation_end) {
      const double sign = elapsed < baseline_seconds_ + pulse_hold_ ? 1.0 : -1.0;
      const double amplitude = channel < 3 ? rate_amplitudes_[channel] : thrust_amplitude_;
      action[channel] += sign * amplitude;
    }
    publishCommand(action, now);
    publishStatus("pulse", currentChannel(), plan_index_);
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
  double thrust_amplitude_{0.015};
  double pulse_hold_{0.35};
  double baseline_seconds_{0.30};
  double recovery_seconds_{2.0};
  int repetitions_{3};
  int plan_index_{0};
  bool active_{false};
  bool recovering_{false};
  ros::Time phase_start_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "brct1order_excite");
  try {
    BrctFirstOrderExcite node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
  return 0;
}
