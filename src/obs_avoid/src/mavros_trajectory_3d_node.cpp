#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class MavrosTrajectory3DNode : public rclcpp::Node
{
public:
  MavrosTrajectory3DNode() : Node("mavros_trajectory_3d_node")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/mavros/trajectory_3d");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    max_points_ = this->declare_parameter<int>("max_points", 5000);
    min_point_distance_ = this->declare_parameter<double>("min_point_distance", 0.02);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort().durability_volatile();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
    path_.header.frame_id = frame_id_;

    RCLCPP_INFO(
      this->get_logger(),
      "mavros_trajectory_3d_node started. odom_topic=%s path_topic=%s frame_id=%s",
      odom_topic_.c_str(), path_topic_.c_str(), frame_id_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    if (!frame_id_.empty()) {
      pose.header.frame_id = frame_id_;
    }

    if (has_last_point_) {
      const double dx = pose.pose.position.x - last_x_;
      const double dy = pose.pose.position.y - last_y_;
      const double dz = pose.pose.position.z - last_z_;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < min_point_distance_) {
        return;
      }
    }

    path_.poses.push_back(pose);
    if (max_points_ > 0 && static_cast<int>(path_.poses.size()) > max_points_) {
      const auto overflow = path_.poses.size() - static_cast<std::size_t>(max_points_);
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<long>(overflow));
    }

    path_.header.stamp = pose.header.stamp;
    path_.header.frame_id = pose.header.frame_id;
    path_pub_->publish(path_);

    last_x_ = pose.pose.position.x;
    last_y_ = pose.pose.position.y;
    last_z_ = pose.pose.position.z;
    has_last_point_ = true;
  }

  std::string odom_topic_;
  std::string path_topic_;
  std::string frame_id_;
  int max_points_{5000};
  double min_point_distance_{0.02};

  bool has_last_point_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};

  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavrosTrajectory3DNode>());
  rclcpp::shutdown();
  return 0;
}
