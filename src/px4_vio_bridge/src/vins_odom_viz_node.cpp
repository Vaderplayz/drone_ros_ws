#include <deque>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class VinsOdomVizNode : public rclcpp::Node {
public:
  VinsOdomVizNode() : Node("vins_odom_viz_node") {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/vins_estimator/odometry");
    path_topic_ = declare_parameter<std::string>("path_topic", "/vins_estimator/trajectory_path");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");
    max_points_ = declare_parameter<int>("max_points", 2000);

    if (max_points_ < 10) {
      RCLCPP_WARN(get_logger(), "max_points too small (%d), clamping to 10", max_points_);
      max_points_ = 10;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(100),
        std::bind(&VinsOdomVizNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "vins_odom_viz_node started odom_topic=%s path_topic=%s", odom_topic_.c_str(),
                path_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    poses_.push_back(pose);
    while (static_cast<int>(poses_.size()) > max_points_) {
      poses_.pop_front();
    }

    nav_msgs::msg::Path path;
    path.header.stamp = msg->header.stamp;
    if (!output_frame_id_.empty()) {
      path.header.frame_id = output_frame_id_;
    } else {
      path.header.frame_id = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    }

    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path);
  }

  std::string odom_topic_;
  std::string path_topic_;
  std::string output_frame_id_;
  int max_points_{};

  std::deque<geometry_msgs::msg::PoseStamped> poses_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VinsOdomVizNode>());
  rclcpp::shutdown();
  return 0;
}
