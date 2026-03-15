// potential_field_planner.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "mavros_msgs/msg/position_target.hpp"
// tf2 dependency for quaternion -> rpy
// we include tf2 headers here:
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <cmath>
#include <vector>
#include <limits>

using namespace std::chrono_literals;

class PotentialFieldPlanner : public rclcpp::Node {
public:
  PotentialFieldPlanner()
  : Node("potential_field_planner")
  {
    // parameters (tune as needed)
    this->declare_parameter("attractive_gain", 1.0);
    this->declare_parameter("repulsive_gain", 1.5);
    this->declare_parameter("repulsive_range", 3.0);
    this->declare_parameter("max_speed", 1.0);
    this->declare_parameter("publish_rate_hz", 10.0);

    this->get_parameter("attractive_gain", k_att_);
    this->get_parameter("repulsive_gain", k_rep_);
    this->get_parameter("repulsive_range", rep_range_);
    this->get_parameter("max_speed", max_speed_);
    double hz; this->get_parameter("publish_rate_hz", hz); publish_period_ = std::chrono::duration<double>(1.0/hz);

    // QoS for sensor-like topics
    auto qos = rclcpp::SensorDataQoS();

    // Subscribers
    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/drone_goal", 10, std::bind(&PotentialFieldPlanner::goal_cb, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos, std::bind(&PotentialFieldPlanner::pose_cb, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan",
      qos, std::bind(&PotentialFieldPlanner::scan_cb, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("planner/cmd_vel", 10);
    position_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // Timer
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(publish_period_),
                                     std::bind(&PotentialFieldPlanner::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "PotentialFieldPlanner started");
  }

private:
  // params
  double k_att_{1.0};
  double k_rep_{1.5};
  double rep_range_{3.0};
  double max_speed_{1.0};
  std::chrono::duration<double> publish_period_;

  // state
  std::mutex mtx_;
  bool have_goal_{false};
  geometry_msgs::msg::Point goal_;
  bool have_pose_{false};
  geometry_msgs::msg::PoseStamped current_pose_;
  bool have_scan_{false};
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_ = *msg;
    have_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%.2f z=%.2f", goal_.x, goal_.y, goal_.z);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    current_pose_ = *msg;
    have_pose_ = true;
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_scan_ = msg;
    have_scan_ = true;
  }

  void control_loop() {
    // copy state under lock
    geometry_msgs::msg::Point goal;
    geometry_msgs::msg::PoseStamped pose;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_goal_ || !have_pose_ || !have_scan_) {
        // not ready
        return;
      }
      goal = goal_;
      pose = current_pose_;
      scan = last_scan_;
    }

    // Current position (x,y)
    double px = pose.pose.position.x;
    double py = pose.pose.position.y;
    double pz = pose.pose.position.z;

    // Attractive force (2D)
    double dx = goal.x - px;
    double dy = goal.y - py;
    double dist_goal = std::hypot(dx, dy);
    // small epsilon to avoid division by zero
    double eps = 1e-6;
    double fax = k_att_ * dx;
    double fay = k_att_ * dy;

    // Repulsive force computed from lidar points within rep_range_
    double frx = 0.0;
    double fry = 0.0;

    // Convert laser scan into obstacle vectors in robot frame,
    // then rotate to local frame using current yaw (assume scan is in base_link frame aligned with world).
    // For SITL/Gazebo the scan angles are in local frame; we assume they are already in the same frame as pose.
    int n = static_cast<int>(scan->ranges.size());
    double angle = scan->angle_min;
    for (int i = 0; i < n; ++i, angle += scan->angle_increment) {
      double r = scan->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r <= 0.01) continue;

      if (r < rep_range_) {
        // obstacle point (in robot frame) relative to robot: (r*cos(angle), r*sin(angle))
        // Map to global frame by rotating with robot yaw.
        // But simpler: if scan angles are in local frame and pose orientation is arbitrary,
        // we can approximate small robot yaw or assume lidar already in local frame aligned to global axes in SITL.
        // For robustness, extract yaw from pose and rotate.
        double roll, pitch, yaw;
        tf2::Quaternion q(
          pose.pose.orientation.x,
          pose.pose.orientation.y,
          pose.pose.orientation.z,
          pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        double ox = px + r * std::cos(yaw + angle);
        double oy = py + r * std::sin(yaw + angle);

        // vector from obstacle to robot
        double vx = px - ox;
        double vy = py - oy;
        double d = std::hypot(vx, vy);
        if (d < 1e-6) continue;

        // repulsive magnitude (simple inverse-squared style with cutoff)
        double coeff = k_rep_ * (1.0 / (d + 1e-6) - 1.0 / rep_range_);
        if (coeff <= 0.0) continue;
        // direction normalized
        frx += coeff * (vx / (d * d)); // stronger when closer (d^2)
        fry += coeff * (vy / (d * d));
      }
    }

    // Total force
    double fx = fax + frx;
    double fy = fay + fry;

    // Convert force into velocity command (scale)
    double vel_x = fx;
    double vel_y = fy;

    // cap speed
    double norm = std::hypot(vel_x, vel_y);
    if (norm > max_speed_) {
      vel_x = vel_x / norm * max_speed_;
      vel_y = vel_y / norm * max_speed_;
    }

    // compute desired yaw: face direction of velocity if moving, else keep current yaw (set angular.z = 0)
    double ang_z = 0.0;
    if (norm > 1e-3) {
      ang_z = std::atan2(vel_y, vel_x); // desired heading
    }

    // Publish TwistStamped for offboard control
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = vel_x;
    twist.twist.linear.y = vel_y;
    twist.twist.linear.z = 0.0; // keep altitude controlled elsewhere
    twist.twist.angular.z = ang_z;
    cmd_vel_pub_->publish(twist);

    // Also publish a MAVROS PositionTarget message with velocity fields (optional)
    mavros_msgs::msg::PositionTarget pt;
    pt.header.stamp = this->now();
    pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    // ignore position, use velocity
    pt.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                   mavros_msgs::msg::PositionTarget::IGNORE_PY |
                   mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    pt.velocity.x = vel_x;
    pt.velocity.y = vel_y;
    pt.velocity.z = 0.0;
    pt.yaw = ang_z;
    position_target_pub_->publish(pt);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PotentialFieldPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
