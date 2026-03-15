// px4_odom_flatten_node.cpp
//
// Subscribes:  /mavros/local_position/odom   (nav_msgs/msg/Odometry)
// Publishes TF: parent="odom", child="base_footprint"
//
// Features:
// - Forces TF frame IDs (doesn't trust msg header frames)
// - Uses msg->header.stamp for TF stamps (best practice)
// - Backward-time prevention with automatic recovery on time reset
// - Does NOT declare use_sim_time (safe to pass via CLI)

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Px4OdomFlattenNode : public rclcpp::Node
{
public:
  Px4OdomFlattenNode()
  : Node("px4_odom_flatten_node"),
    have_last_stamp_(false)
  {
    // User parameters
    this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("child_frame", "base_footprint");

    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("parent_frame", parent_frame_);
    this->get_parameter("child_frame", child_frame_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4OdomFlattenNode::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
      "Subscribed to: %s | Publishing TF: %s -> %s",
      odom_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Drop zero timestamps
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping odom: zero timestamp (0,0) from %s", odom_topic_.c_str()
      );
      return;
    }

    // IMPORTANT: Use the stamp directly from the message.
    // This avoids mixing ROS time vs system time domains.
    const rclcpp::Time stamp(msg->header.stamp);

    // Backward-time handling:
    // - If time goes backwards slightly: drop
    // - If time resets by a lot (sim reset): accept new timeline by resetting last_stamp_
    if (have_last_stamp_) {
      if (stamp < last_stamp_) {
        const double dt = (last_stamp_ - stamp).seconds();

        // If the clock reset is big (e.g. sim restarted), resync instead of dropping forever.
        if (dt > 5.0) {
          RCLCPP_WARN(this->get_logger(),
            "Detected time reset/backward jump (%.3fs). Resyncing TF timeline. last=%.6f now=%.6f",
            dt, last_stamp_.seconds(), stamp.seconds());
          last_stamp_ = stamp;
          // Continue to publish this transform.
        } else {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Dropping odom with slight backwards time. stamp=%.6f < last=%.6f (dt=%.3fs, topic=%s)",
            stamp.seconds(), last_stamp_.seconds(), dt, odom_topic_.c_str()
          );
          return;
        }
      } else {
        last_stamp_ = stamp;
      }
    } else {
      last_stamp_ = stamp;
      have_last_stamp_ = true;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;

    // Force frames as requested
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id  = child_frame_;

    // Pose from odometry
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation      = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_stamp_;
  bool have_last_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdomFlattenNode>());
  rclcpp::shutdown();
  return 0;
}
