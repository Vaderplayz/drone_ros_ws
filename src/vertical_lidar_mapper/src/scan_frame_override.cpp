// Timestamp: 2026-02-21 11:18:51 +07+0700
// Most Recent Update: Unified vertical and horizontal frame overrides into one node
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace vertical_lidar_mapper
{

class ScanFrameOverrideNode : public rclcpp::Node
{
public:
  ScanFrameOverrideNode()
  : rclcpp::Node("scan_frame_override")
  {
    enable_vertical_override_ = this->declare_parameter<bool>("enable_vertical_override", true);
    vertical_input_topic_ = this->declare_parameter<std::string>(
      "vertical_input_topic",
      "/world/walls/model/x500_lidar_2d_tilted_0/model/lidar_vert/link/link/sensor/lidar_2d_v2/scan");
    vertical_output_topic_ = this->declare_parameter<std::string>("vertical_output_topic", "/scan_vertical");
    vertical_override_frame_id_ = this->declare_parameter<std::string>("vertical_override_frame_id", "lidar_vert_link");
    vertical_force_override_ = this->declare_parameter<bool>("vertical_force_override", true);

    enable_horizontal_override_ = this->declare_parameter<bool>("enable_horizontal_override", true);
    horizontal_input_topic_ = this->declare_parameter<std::string>(
      "horizontal_input_topic",
      "/world/walls/model/x500_lidar_2d_tilted_0/model/lidar_horiz/link/link/sensor/lidar_2d_v2/scan");
    horizontal_output_topic_ = this->declare_parameter<std::string>("horizontal_output_topic", "/scan_horizontal");
    horizontal_override_frame_id_ = this->declare_parameter<std::string>("horizontal_override_frame_id", "lidar_horiz_link");
    horizontal_force_override_ = this->declare_parameter<bool>("horizontal_force_override", true);

    if (enable_vertical_override_) {
      vertical_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        vertical_output_topic_,
        rclcpp::SensorDataQoS());

      vertical_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        vertical_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ScanFrameOverrideNode::verticalCallback, this, std::placeholders::_1));
    }

    if (enable_horizontal_override_) {
      horizontal_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        horizontal_output_topic_,
        rclcpp::SensorDataQoS());

      horizontal_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        horizontal_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ScanFrameOverrideNode::horizontalCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "scan_frame_override started. vertical_enabled=%s horizontal_enabled=%s",
      enable_vertical_override_ ? "true" : "false",
      enable_horizontal_override_ ? "true" : "false");

    if (enable_vertical_override_) {
      RCLCPP_INFO(
        this->get_logger(),
        "vertical: input='%s' output='%s' frame='%s' force_override=%s",
        vertical_input_topic_.c_str(),
        vertical_output_topic_.c_str(),
        vertical_override_frame_id_.c_str(),
        vertical_force_override_ ? "true" : "false");
    }

    if (enable_horizontal_override_) {
      RCLCPP_INFO(
        this->get_logger(),
        "horizontal: input='%s' output='%s' frame='%s' force_override=%s",
        horizontal_input_topic_.c_str(),
        horizontal_output_topic_.c_str(),
        horizontal_override_frame_id_.c_str(),
        horizontal_force_override_ ? "true" : "false");
    }

    if (!enable_vertical_override_ && !enable_horizontal_override_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Both overrides are disabled. Node will remain idle.");
    }
  }

private:
  void processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & pub,
    const std::string & override_frame_id,
    const bool force_override,
    const char * stream_name)
  {
    if (!pub) {
      return;
    }

    sensor_msgs::msg::LaserScan out = *msg;

    if (override_frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s override frame is empty. Passing through original frame_id='%s'.",
        stream_name,
        msg->header.frame_id.c_str());
      pub->publish(out);
      return;
    }

    if (force_override || out.header.frame_id.empty()) {
      out.header.frame_id = override_frame_id;
    }

    pub->publish(out);
  }

  void verticalCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    processScan(msg, vertical_pub_, vertical_override_frame_id_, vertical_force_override_, "vertical");
  }

  void horizontalCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    processScan(msg, horizontal_pub_, horizontal_override_frame_id_, horizontal_force_override_, "horizontal");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr vertical_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr vertical_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr horizontal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr horizontal_pub_;

  bool enable_vertical_override_{true};
  std::string vertical_input_topic_;
  std::string vertical_output_topic_;
  std::string vertical_override_frame_id_;
  bool vertical_force_override_{true};

  bool enable_horizontal_override_{true};
  std::string horizontal_input_topic_;
  std::string horizontal_output_topic_;
  std::string horizontal_override_frame_id_;
  bool horizontal_force_override_{true};
};

}  // namespace vertical_lidar_mapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vertical_lidar_mapper::ScanFrameOverrideNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
