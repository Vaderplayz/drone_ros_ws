// Timestamp: 2026-02-21 11:18:02 +07+0700
// Most Recent Update: Metadata timestamp now uses local time
#include <algorithm>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace vertical_lidar_mapper
{

class OdomToTfBridge : public rclcpp::Node
{
public:
  OdomToTfBridge()
  : rclcpp::Node("odom_to_tf_bridge")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    use_msg_frame_ids_ = this->declare_parameter<bool>("use_msg_frame_ids", true);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomToTfBridge::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "odom_to_tf_bridge started. odom_topic='%s', odom_frame='%s', base_frame='%s'",
      odom_topic_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  static std::string stripLeadingSlash(const std::string & frame)
  {
    if (!frame.empty() && frame.front() == '/') {
      return frame.substr(1);
    }
    return frame;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    std::string odom_frame = odom_frame_;
    std::string base_frame = base_frame_;

    if (use_msg_frame_ids_) {
      if (!odom_msg->header.frame_id.empty()) {
        odom_frame = odom_msg->header.frame_id;
      }
      if (!odom_msg->child_frame_id.empty()) {
        base_frame = odom_msg->child_frame_id;
      }
    }

    odom_frame = stripLeadingSlash(odom_frame);
    base_frame = stripLeadingSlash(base_frame);

    if (odom_frame.empty() || base_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot publish TF because frame ids are empty.");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg->header.stamp;
    if (tf_msg.header.stamp.sec == 0 && tf_msg.header.stamp.nanosec == 0) {
      tf_msg.header.stamp = this->now();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Odometry stamp is zero, publishing TF with node time now().");
    }

    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
    tf_msg.transform.translation.x = odom_msg->pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg->pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg->pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg->pose.pose.orientation;

    const auto & q = tf_msg.transform.rotation;
    const bool invalid_quat =
      (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0);
    if (invalid_quat) {
      tf_msg.transform.rotation.w = 1.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Odometry orientation quaternion was zero; replaced with identity.");
    }

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool use_msg_frame_ids_{true};
};

}  // namespace vertical_lidar_mapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vertical_lidar_mapper::OdomToTfBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
