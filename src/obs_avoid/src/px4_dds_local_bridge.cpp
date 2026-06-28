// Converts PX4 DDS local position and GPS topics into standard ROS messages.

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.h"

class Px4DdsLocalBridge : public rclcpp::Node {
 public:
  Px4DdsLocalBridge() : Node("px4_dds_local_bridge") {
    px4_local_position_topic_ =
        declare_parameter<std::string>("px4_local_position_topic", "/fmu/out/vehicle_local_position");
    px4_gps_topic_ = declare_parameter<std::string>("px4_gps_topic", "/fmu/out/vehicle_gps_position");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/px4/odom");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/px4/local_position/pose");
    gps_fix_topic_ = declare_parameter<std::string>("gps_fix_topic", "/px4/gps/fix");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    auto sensor_qos = rclcpp::SensorDataQoS();
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    pub_gps_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_fix_topic_, 10);
    sub_local_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_local_position_topic_, sensor_qos,
        std::bind(&Px4DdsLocalBridge::on_local_position, this, std::placeholders::_1));
    sub_gps_ = create_subscription<px4_msgs::msg::SensorGps>(
        px4_gps_topic_, sensor_qos, std::bind(&Px4DdsLocalBridge::on_gps, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PX4 DDS bridge started: %s -> %s, %s -> %s",
                px4_local_position_topic_.c_str(), odom_topic_.c_str(), px4_gps_topic_.c_str(),
                gps_fix_topic_.c_str());
  }

 private:
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    const auto stamp = now();
    const double x_enu = msg->y;
    const double y_enu = msg->x;
    const double z_enu = -msg->z;
    const double vx_enu = msg->vy;
    const double vy_enu = msg->vx;
    const double vz_enu = -msg->vz;
    const double yaw_enu = (M_PI_2 - static_cast<double>(msg->heading));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_enu);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_enu;
    odom.pose.pose.position.y = y_enu;
    odom.pose.pose.position.z = z_enu;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx_enu;
    odom.twist.twist.linear.y = vy_enu;
    odom.twist.twist.linear.z = vz_enu;
    pub_odom_->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pub_pose_->publish(pose);
  }

  void on_gps(const px4_msgs::msg::SensorGps::SharedPtr msg) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = now();
    fix.header.frame_id = "gps";
    fix.status.status = msg->fix_type >= 3 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                            : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = msg->latitude_deg;
    fix.longitude = msg->longitude_deg;
    fix.altitude = msg->altitude_msl_m;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    const double eph = std::max(0.01F, msg->eph);
    const double epv = std::max(0.01F, msg->epv);
    fix.position_covariance[0] = eph * eph;
    fix.position_covariance[4] = eph * eph;
    fix.position_covariance[8] = epv * epv;
    pub_gps_->publish(fix);
  }

  std::string px4_local_position_topic_;
  std::string px4_gps_topic_;
  std::string odom_topic_;
  std::string pose_topic_;
  std::string gps_fix_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_gps_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4DdsLocalBridge>());
  rclcpp::shutdown();
  return 0;
}
