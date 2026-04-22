#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class Nav2CmdVelBridge : public rclcpp::Node {
 public:
  Nav2CmdVelBridge() : Node("nav2_cmd_vel_bridge") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/planner_cmd_vel");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    auto qos = rclcpp::QoS(20).reliable();
    pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, qos);
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, qos, std::bind(&Nav2CmdVelBridge::on_cmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "nav2_cmd_vel_bridge started: %s -> %s (frame_id=%s)",
                input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

 private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = frame_id_;
    out.twist = *msg;
    pub_cmd_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
