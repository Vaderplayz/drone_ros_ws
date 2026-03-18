#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/companion_process_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
inline bool stamp_is_zero(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0;
}
}  // namespace

class VioToMavrosBridgeNode : public rclcpp::Node {
 public:
  VioToMavrosBridgeNode() : Node("vio_to_mavros_bridge") {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/vins_estimator/odometry");
    input_pose_topic_ = declare_parameter<std::string>("input_pose_topic", "");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/mavros/odometry/out");
    companion_status_topic_ = declare_parameter<std::string>("companion_status_topic", "/mavros/companion_process/status");

    output_parent_frame_id_ = declare_parameter<std::string>("output_parent_frame_id", "odom");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "base_link");

    keep_input_stamp_ = declare_parameter<bool>("keep_input_stamp", true);
    status_rate_hz_ = declare_parameter<double>("status_rate_hz", 2.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    termination_timeout_sec_ = declare_parameter<double>("termination_timeout_sec", 2.0);
    startup_boot_sec_ = declare_parameter<double>("startup_boot_sec", 1.5);

    stale_timeout_sec_ = std::max(0.05, stale_timeout_sec_);
    termination_timeout_sec_ = std::max(stale_timeout_sec_, termination_timeout_sec_);
    startup_boot_sec_ = std::max(0.0, startup_boot_sec_);
    status_rate_hz_ = std::max(0.2, status_rate_hz_);

    pub_odom_out_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    pub_companion_status_ =
        create_publisher<mavros_msgs::msg::CompanionProcessStatus>(companion_status_topic_, 10);

    sub_odom_in_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VioToMavrosBridgeNode::odomCallback, this, std::placeholders::_1));

    if (!input_pose_topic_.empty()) {
      sub_pose_in_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          input_pose_topic_, rclcpp::SensorDataQoS(),
          std::bind(&VioToMavrosBridgeNode::poseCallback, this, std::placeholders::_1));
    }

    const auto status_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(status_period),
        std::bind(&VioToMavrosBridgeNode::publishCompanionStatus, this));

    startup_time_ = now();

    RCLCPP_INFO(
        get_logger(),
        "px4_vio_bridge started odom_in=%s pose_in=%s out=%s status=%s frame=[%s -> %s]",
        input_odom_topic_.c_str(), input_pose_topic_.empty() ? "(disabled)" : input_pose_topic_.c_str(),
        output_odom_topic_.c_str(), companion_status_topic_.c_str(), output_parent_frame_id_.c_str(),
        output_child_frame_id_.c_str());

    RCLCPP_INFO(
        get_logger(),
        "Reference mapping: OpenVINS odometry -> /mavros/odometry/out, companion status component=%u",
        mavros_msgs::msg::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
  }

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav_msgs::msg::Odometry out = *msg;

    if (!keep_input_stamp_ || stamp_is_zero(out.header.stamp)) {
      out.header.stamp = now();
    }

    // MAVROS odometry plugin follows REP-147 and forwards to PX4 estimator.
    // We force explicit frame ids so upstream VIO frame naming is consistent.
    out.header.frame_id = output_parent_frame_id_;
    out.child_frame_id = output_child_frame_id_;

    pub_odom_out_->publish(out);

    last_input_rx_time_ = now();
    got_odom_ = true;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    nav_msgs::msg::Odometry out;

    if (!keep_input_stamp_ || stamp_is_zero(msg->header.stamp)) {
      out.header.stamp = now();
    } else {
      out.header.stamp = msg->header.stamp;
    }
    out.header.frame_id = output_parent_frame_id_;
    out.child_frame_id = output_child_frame_id_;
    out.pose.pose = msg->pose;

    pub_odom_out_->publish(out);

    last_input_rx_time_ = now();
    got_odom_ = true;
  }

  uint8_t computeState() const {
    if (!got_odom_) {
      const double since_start = (now() - startup_time_).seconds();
      if (since_start <= startup_boot_sec_) {
        return mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_BOOT;
      }
      return mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_CRITICAL;
    }

    const double age = (now() - last_input_rx_time_).seconds();
    if (age <= stale_timeout_sec_) {
      return mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_ACTIVE;
    }
    if (age <= termination_timeout_sec_) {
      return mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_CRITICAL;
    }
    return mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;
  }

  void publishCompanionStatus() {
    mavros_msgs::msg::CompanionProcessStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = output_parent_frame_id_;
    msg.component = mavros_msgs::msg::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    msg.state = computeState();

    if (msg.state != last_state_) {
      RCLCPP_INFO(
          get_logger(),
          "VIO companion state changed: %u -> %u (0=UNINIT,1=BOOT,3=STANDBY,4=ACTIVE,5=CRITICAL,8=FLIGHT_TERMINATION)",
          last_state_, msg.state);
      last_state_ = msg.state;
    }

    pub_companion_status_->publish(msg);
  }

  std::string input_odom_topic_;
  std::string input_pose_topic_;
  std::string output_odom_topic_;
  std::string companion_status_topic_;
  std::string output_parent_frame_id_;
  std::string output_child_frame_id_;

  bool keep_input_stamp_{true};
  double status_rate_hz_{2.0};
  double stale_timeout_sec_{0.5};
  double termination_timeout_sec_{2.0};
  double startup_boot_sec_{1.5};

  bool got_odom_{false};
  uint8_t last_state_{mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_UNINIT};

  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_rx_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_in_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_in_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_out_;
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr pub_companion_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioToMavrosBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
