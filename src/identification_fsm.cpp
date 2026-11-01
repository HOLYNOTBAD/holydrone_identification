#include <ros/ros.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <string>

class IdentificationFsm {
 public:
  IdentificationFsm() : nh_(), private_nh_("~") {
    private_nh_.param<std::string>("exit_mode", exit_mode_, "POSCTL");
    private_nh_.param<std::string>("exciter", exciter_, "brct1order_excite");
    const std::string prefix = "/" + exciter_;
    state_pub_ = nh_.advertise<std_msgs::String>("/identification/state", 1, true);
    state_sub_ = nh_.subscribe("/mavros/state", 10, &IdentificationFsm::stateCallback, this);
    finished_sub_ = nh_.subscribe(prefix + "/finished", 1,
                                  &IdentificationFsm::finishedCallback, this);
    start_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix + "/start");
    stop_client_ = nh_.serviceClient<std_srvs::Trigger>(prefix + "/stop");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    retry_timer_ = nh_.createTimer(ros::Duration(0.5), &IdentificationFsm::retryExit, this);
    publishState("WAIT_OFFBOARD");
  }

 private:
  void publishState(const std::string& state) {
    std_msgs::String message;
    message.data = state;
    state_pub_.publish(message);
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    last_state_ = *message;
    const bool offboard = message->connected && message->armed && message->mode == "OFFBOARD";

    if (offboard && !was_offboard_ && !waiting_for_exit_) {
      std_srvs::Trigger request;
      if (start_client_.call(request) && request.response.success) {
        active_ = true;
        publishState("EXCITING");
        ROS_INFO("OFFBOARD entered manually: %s started", exciter_.c_str());
      } else {
        publishState("ERROR");
        ROS_ERROR("Could not start %s: %s", exciter_.c_str(), request.response.message.c_str());
      }
    } else if (!offboard && was_offboard_) {
      if (active_) {
        std_srvs::Trigger request;
        stop_client_.call(request);
        ROS_INFO("OFFBOARD exited manually: BRCT identification stopped");
      }
      active_ = false;
      waiting_for_exit_ = false;
      publishState("WAIT_OFFBOARD");
    }
    was_offboard_ = offboard;
  }

  void finishedCallback(const std_msgs::Bool::ConstPtr& message) {
    if (!message->data || !active_) {
      return;
    }
    active_ = false;
    waiting_for_exit_ = true;
    publishState("COMPLETE");
    ROS_INFO("%s completed; requesting mode %s", exciter_.c_str(), exit_mode_.c_str());
    requestExitMode();
  }

  void requestExitMode() {
    if (!last_state_.connected || last_state_.mode != "OFFBOARD") {
      return;
    }
    mavros_msgs::SetMode request;
    request.request.custom_mode = exit_mode_;
    if (!mode_client_.call(request) || !request.response.mode_sent) {
      ROS_WARN_THROTTLE(2.0, "Waiting to exit OFFBOARD to %s", exit_mode_.c_str());
    }
  }

  void retryExit(const ros::TimerEvent&) {
    if (waiting_for_exit_) {
      requestExitMode();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher state_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber finished_sub_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient mode_client_;
  ros::Timer retry_timer_;
  mavros_msgs::State last_state_;
  std::string exit_mode_;
  std::string exciter_;
  bool active_{false};
  bool was_offboard_{false};
  bool waiting_for_exit_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "identification_fsm");
  IdentificationFsm node;
  ros::spin();
  return 0;
}
