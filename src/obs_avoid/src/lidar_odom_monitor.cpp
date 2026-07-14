#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string bool_string(bool value)
{
  return value ? "true" : "false";
}

bool finite_pose(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}
}  // namespace

class LidarOdomMonitor : public rclcpp::Node
{
public:
  LidarOdomMonitor()
  : Node("lidar_odom_monitor")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_rf2o");
    scan_diagnostics_topic_ = declare_parameter<std::string>(
      "scan_diagnostics_topic", "/scan_rf2o/diagnostics");
    raw_topic_ = declare_parameter<std::string>("raw_odom_topic", "/lidar/odom_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    output_topic_ = declare_parameter<std::string>("output_odom_topic", "/lidar/odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/lidar_odom/diagnostics");
    health_csv_path_ = declare_parameter<std::string>("health_csv_path", "");
    expected_parent_frame_ = declare_parameter<std::string>("expected_parent_frame", "lidar_odom");
    expected_child_frame_ = declare_parameter<std::string>("expected_child_frame", "base_footprint");
    expected_scan_bins_ = declare_parameter<int>("expected_scan_bins", 720);
    minimum_angular_coverage_ratio_ = declare_parameter<double>(
      "minimum_angular_coverage_ratio", 0.70);
    minimum_finite_return_ratio_ = declare_parameter<double>(
      "minimum_finite_return_ratio", 0.05);
    minimum_scan_rate_hz_ = declare_parameter<double>("minimum_scan_rate_hz", 1.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.50);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.50);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.50);
    scan_diagnostics_timeout_sec_ = declare_parameter<double>(
      "scan_diagnostics_timeout_sec", 1.50);
    max_translation_step_m_ = declare_parameter<double>("max_translation_step_m", 0.25);
    max_yaw_step_rad_ = declare_parameter<double>("max_yaw_step_deg", 15.0) * kDegToRad;
    max_translation_speed_mps_ = declare_parameter<double>("max_translation_speed_mps", 1.0);
    max_yaw_rate_radps_ = declare_parameter<double>("max_yaw_rate_degps", 120.0) * kDegToRad;
    max_roll_rad_ = declare_parameter<double>("max_roll_deg", 10.0) * kDegToRad;
    max_pitch_rad_ = declare_parameter<double>("max_pitch_deg", 10.0) * kDegToRad;
    max_abs_z_m_ = declare_parameter<double>("max_abs_z_m", 0.05);
    required_consecutive_valid_odom_ = declare_parameter<int>(
      "required_consecutive_valid_odom", 5);
    health_publish_hz_ = declare_parameter<double>("health_publish_hz", 1.0);

    validate_parameters();
    open_health_csv();
    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomMonitor::scan_callback, this, std::placeholders::_1));
    scan_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      scan_diagnostics_topic_, rclcpp::QoS(10),
      std::bind(&LidarOdomMonitor::scan_diagnostics_callback, this, std::placeholders::_1));
    raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&LidarOdomMonitor::odom_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomMonitor::imu_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / health_publish_hz_)),
      std::bind(&LidarOdomMonitor::publish_diagnostics, this));

    transition("WAITING_FOR_SCAN");
    RCLCPP_INFO(
      get_logger(), "Monitoring coherent scan %s and RF2O %s -> %s",
      scan_topic_.c_str(), raw_topic_.c_str(), output_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (expected_scan_bins_ < 2 || minimum_angular_coverage_ratio_ <= 0.0 ||
      minimum_angular_coverage_ratio_ > 1.0 || minimum_finite_return_ratio_ < 0.0 ||
      minimum_finite_return_ratio_ > 1.0 || minimum_scan_rate_hz_ <= 0.0 ||
      scan_timeout_sec_ <= 0.0 || odom_timeout_sec_ <= 0.0 || imu_timeout_sec_ <= 0.0 ||
      scan_diagnostics_timeout_sec_ <= 0.0 || max_translation_step_m_ <= 0.0 ||
      max_yaw_step_rad_ <= 0.0 || max_translation_speed_mps_ <= 0.0 ||
      max_yaw_rate_radps_ <= 0.0 || max_roll_rad_ <= 0.0 || max_pitch_rad_ <= 0.0 ||
      max_abs_z_m_ <= 0.0 || required_consecutive_valid_odom_ < 1 || health_publish_hz_ <= 0.0)
    {
      throw std::invalid_argument("Invalid LiDAR odometry monitor parameters");
    }
  }

  void open_health_csv()
  {
    if (health_csv_path_.empty()) {
      return;
    }
    health_csv_.open(health_csv_path_, std::ios::out | std::ios::app);
    if (!health_csv_) {
      throw std::runtime_error("Cannot open LiDAR odometry health CSV: " + health_csv_path_);
    }
    health_csv_.seekp(0, std::ios::end);
    if (health_csv_.tellp() == 0) {
      health_csv_ << "timestamp,raw_odom_age_sec,scan_age_sec,rf2o_rate_hz,scan_rate_hz,x,y,"
        "yaw_deg,delta_translation_m,delta_yaw_deg,translation_speed_mps,yaw_rate_degps,"
        "roll_deg,pitch_deg,valid,status_reason\n";
      health_csv_.flush();
    }
  }

  double age(const rclcpp::Time & receipt) const
  {
    return receipt.nanoseconds() > 0 ? (now() - receipt).seconds() :
      std::numeric_limits<double>::infinity();
  }

  void update_rate(const rclcpp::Time & receipt, rclcpp::Time & previous, double & rate)
  {
    if (previous.nanoseconds() > 0) {
      const double period = (receipt - previous).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instantaneous = 1.0 / period;
        rate = rate > 0.0 ? 0.8 * rate + 0.2 * instantaneous : instantaneous;
      }
    }
    previous = receipt;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto receipt = now();
    update_rate(receipt, previous_scan_receipt_, scan_rate_hz_);
    scan_receipt_ = receipt;
    scan_length_ = msg->ranges.size();
    const rclcpp::Time stamp(msg->header.stamp);
    scan_timestamp_monotonic_ = stamp.nanoseconds() > 0 &&
      (!have_scan_stamp_ || stamp > last_scan_stamp_);
    if (scan_timestamp_monotonic_) {
      last_scan_stamp_ = stamp;
      have_scan_stamp_ = true;
    }
    std::size_t finite_count = 0;
    for (const float range : msg->ranges) {
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
        ++finite_count;
      }
    }
    direct_finite_return_ratio_ = msg->ranges.empty() ? 0.0 :
      static_cast<double>(finite_count) / static_cast<double>(msg->ranges.size());
  }

  void scan_diagnostics_callback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for (const auto & status : msg->status) {
      if (status.name != "laser_scan_canonicalizer") {
        continue;
      }
      canonical_diagnostics_ok_ = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
      for (const auto & value : status.values) {
        try {
          if (value.key == "angular_observation_coverage_ratio") {
            angular_coverage_ratio_ = std::stod(value.value);
          } else if (value.key == "finite_return_ratio") {
            diagnostic_finite_return_ratio_ = std::stod(value.value);
          }
        } catch (const std::exception &) {
          canonical_diagnostics_ok_ = false;
        }
      }
      scan_diagnostics_receipt_ = now();
      return;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    const double norm = quaternion.length2();
    if (!std::isfinite(norm) || norm < std::numeric_limits<double>::epsilon()) {
      imu_valid_ = false;
      imu_receipt_ = now();
      return;
    }
    quaternion.normalize();
    double unused_yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, unused_yaw);
    imu_valid_ = std::isfinite(imu_roll_) && std::isfinite(imu_pitch_);
    imu_receipt_ = now();
  }

  std::string non_odom_state() const
  {
    if (scan_receipt_.nanoseconds() <= 0) {
      return "WAITING_FOR_SCAN";
    }
    if (age(scan_receipt_) > scan_timeout_sec_) {
      return "STALE_SCAN";
    }
    if (!scan_timestamp_monotonic_) {
      return "NON_MONOTONIC_TIMESTAMP";
    }
    if (scan_length_ != static_cast<std::size_t>(expected_scan_bins_) ||
      age(scan_diagnostics_receipt_) > scan_diagnostics_timeout_sec_ ||
      !canonical_diagnostics_ok_ || angular_coverage_ratio_ < minimum_angular_coverage_ratio_)
    {
      return "LOW_ANGULAR_COVERAGE";
    }
    if (std::min(direct_finite_return_ratio_, diagnostic_finite_return_ratio_) <
      minimum_finite_return_ratio_)
    {
      return "LOW_FINITE_RETURN_RATIO";
    }
    if (scan_rate_hz_ < minimum_scan_rate_hz_) {
      return "STALE_SCAN";
    }
    if (imu_receipt_.nanoseconds() <= 0 || age(imu_receipt_) > imu_timeout_sec_ || !imu_valid_) {
      return "WAITING_FOR_ODOM";
    }
    if (std::abs(imu_roll_) > max_roll_rad_) {
      return "EXCESSIVE_ROLL";
    }
    if (std::abs(imu_pitch_) > max_pitch_rad_) {
      return "EXCESSIVE_PITCH";
    }
    return "OK";
  }

  bool extract_yaw(const nav_msgs::msg::Odometry & msg, double & yaw) const
  {
    const auto & q = msg.pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    const double norm = quaternion.length2();
    if (!std::isfinite(norm) || norm < std::numeric_limits<double>::epsilon()) {
      return false;
    }
    quaternion.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return std::isfinite(yaw);
  }

  void reject_odom(const std::string & reason)
  {
    odom_valid_ = false;
    odom_reason_ = reason;
    consecutive_valid_odom_ = 0;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto receipt = now();
    update_rate(receipt, previous_odom_receipt_, rf2o_rate_hz_);
    odom_receipt_ = receipt;
    if (msg->header.frame_id != expected_parent_frame_ ||
      msg->child_frame_id != expected_child_frame_ || !finite_pose(*msg) ||
      std::abs(msg->pose.pose.position.z) > max_abs_z_m_)
    {
      reject_odom("NONFINITE_ODOMETRY");
      return;
    }

    double yaw = 0.0;
    if (!extract_yaw(*msg, yaw)) {
      reject_odom("INVALID_QUATERNION");
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0 || (have_odom_stamp_ && stamp <= last_odom_stamp_)) {
      reject_odom("NON_MONOTONIC_TIMESTAMP");
      return;
    }

    delta_translation_m_ = 0.0;
    delta_yaw_rad_ = 0.0;
    translation_speed_mps_ = 0.0;
    yaw_rate_radps_ = 0.0;
    if (have_odom_pose_) {
      const double dt = (stamp - last_odom_stamp_).seconds();
      if (dt <= 0.0 || !std::isfinite(dt)) {
        reject_odom("NON_MONOTONIC_TIMESTAMP");
        return;
      }
      delta_translation_m_ = std::hypot(
        msg->pose.pose.position.x - last_x_, msg->pose.pose.position.y - last_y_);
      delta_yaw_rad_ = std::abs(wrap_pi(yaw - last_yaw_));
      translation_speed_mps_ = delta_translation_m_ / dt;
      yaw_rate_radps_ = delta_yaw_rad_ / dt;
      if (delta_translation_m_ > max_translation_step_m_) {
        reject_odom("TRANSLATION_JUMP");
        return;
      }
      if (delta_yaw_rad_ > max_yaw_step_rad_) {
        reject_odom("YAW_JUMP");
        return;
      }
      if (translation_speed_mps_ > max_translation_speed_mps_) {
        reject_odom("EXCESSIVE_TRANSLATION_SPEED");
        return;
      }
      if (yaw_rate_radps_ > max_yaw_rate_radps_) {
        reject_odom("EXCESSIVE_YAW_RATE");
        return;
      }
    }

    const std::string upstream_state = non_odom_state();
    if (upstream_state != "OK") {
      reject_odom(upstream_state);
      return;
    }

    last_odom_stamp_ = stamp;
    have_odom_stamp_ = true;
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;
    last_yaw_ = yaw;
    have_odom_pose_ = true;
    latest_x_ = last_x_;
    latest_y_ = last_y_;
    latest_yaw_ = last_yaw_;
    odom_valid_ = true;
    odom_reason_ = "OK";
    ++consecutive_valid_odom_;

    if (consecutive_valid_odom_ >= static_cast<std::size_t>(required_consecutive_valid_odom_)) {
      nav_msgs::msg::Odometry output = *msg;
      output.pose.pose.position.z = 0.0;
      output_pub_->publish(output);
    }
  }

  std::string current_state() const
  {
    const std::string upstream = non_odom_state();
    if (upstream != "OK") {
      return upstream;
    }
    if (odom_receipt_.nanoseconds() <= 0) {
      return "WAITING_FOR_ODOM";
    }
    if (!odom_valid_) {
      return odom_reason_;
    }
    if (age(odom_receipt_) > odom_timeout_sec_) {
      return "STALE_ODOM";
    }
    if (consecutive_valid_odom_ < static_cast<std::size_t>(required_consecutive_valid_odom_)) {
      return "WAITING_FOR_ODOM";
    }
    return "OK";
  }

  void transition(const std::string & state)
  {
    if (state == state_) {
      return;
    }
    if (state == "OK") {
      RCLCPP_INFO(get_logger(), "RF2O monitor state: OK");
    } else {
      RCLCPP_WARN(get_logger(), "RF2O monitor state: %s", state.c_str());
    }
    state_ = state;
  }

  void write_health_csv(bool healthy)
  {
    if (!health_csv_) {
      return;
    }
    health_csv_ << std::fixed << std::setprecision(9) << now().seconds() << ','
                << age(odom_receipt_) << ',' << age(scan_receipt_) << ',' << rf2o_rate_hz_ << ','
                << scan_rate_hz_ << ',' << latest_x_ << ',' << latest_y_ << ','
                << latest_yaw_ * kRadToDeg << ',' << delta_translation_m_ << ','
                << delta_yaw_rad_ * kRadToDeg << ',' << translation_speed_mps_ << ','
                << yaw_rate_radps_ * kRadToDeg << ',' << imu_roll_ * kRadToDeg << ','
                << imu_pitch_ * kRadToDeg << ',' << bool_string(healthy) << ',' << state_ << '\n';
    health_csv_.flush();
  }

  void publish_diagnostics()
  {
    transition(current_state());
    const bool healthy = state_ == "OK";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "lidar_odom_monitor";
    status.hardware_id = "rf2o_laser_odometry";
    status.message = state_;
    status.values.push_back(key_value("valid", bool_string(healthy)));
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value("scan_age_sec", std::to_string(age(scan_receipt_))));
    status.values.push_back(key_value("odom_age_sec", std::to_string(age(odom_receipt_))));
    status.values.push_back(key_value("scan_rate_hz", std::to_string(scan_rate_hz_)));
    status.values.push_back(key_value("rf2o_rate_hz", std::to_string(rf2o_rate_hz_)));
    status.values.push_back(key_value("scan_length", std::to_string(scan_length_)));
    status.values.push_back(key_value(
      "angular_coverage_ratio", std::to_string(angular_coverage_ratio_)));
    status.values.push_back(key_value(
      "finite_return_ratio", std::to_string(direct_finite_return_ratio_)));
    status.values.push_back(key_value(
      "consecutive_valid_odom", std::to_string(consecutive_valid_odom_)));
    status.values.push_back(key_value("delta_translation_m", std::to_string(delta_translation_m_)));
    status.values.push_back(key_value("delta_yaw_deg", std::to_string(delta_yaw_rad_ * kRadToDeg)));
    status.values.push_back(key_value(
      "translation_speed_mps", std::to_string(translation_speed_mps_)));
    status.values.push_back(key_value("yaw_rate_degps", std::to_string(yaw_rate_radps_ * kRadToDeg)));
    status.values.push_back(key_value("roll_deg", std::to_string(imu_roll_ * kRadToDeg)));
    status.values.push_back(key_value("pitch_deg", std::to_string(imu_pitch_ * kRadToDeg)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
    write_health_csv(healthy);
  }

  std::string scan_topic_;
  std::string scan_diagnostics_topic_;
  std::string raw_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string health_csv_path_;
  std::string expected_parent_frame_;
  std::string expected_child_frame_;
  int expected_scan_bins_{720};
  int required_consecutive_valid_odom_{5};
  double minimum_angular_coverage_ratio_{0.70};
  double minimum_finite_return_ratio_{0.05};
  double minimum_scan_rate_hz_{1.0};
  double scan_timeout_sec_{0.50};
  double odom_timeout_sec_{0.50};
  double imu_timeout_sec_{0.50};
  double scan_diagnostics_timeout_sec_{1.50};
  double max_translation_step_m_{0.25};
  double max_yaw_step_rad_{15.0 * kDegToRad};
  double max_translation_speed_mps_{1.0};
  double max_yaw_rate_radps_{120.0 * kDegToRad};
  double max_roll_rad_{10.0 * kDegToRad};
  double max_pitch_rad_{10.0 * kDegToRad};
  double max_abs_z_m_{0.05};
  double health_publish_hz_{1.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr scan_diagnostics_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::ofstream health_csv_;

  rclcpp::Time scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_diagnostics_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool have_scan_stamp_{false};
  bool have_odom_stamp_{false};
  bool have_odom_pose_{false};
  bool scan_timestamp_monotonic_{true};
  bool canonical_diagnostics_ok_{false};
  bool imu_valid_{false};
  bool odom_valid_{false};
  std::size_t scan_length_{0};
  std::size_t consecutive_valid_odom_{0};
  double scan_rate_hz_{0.0};
  double rf2o_rate_hz_{0.0};
  double angular_coverage_ratio_{0.0};
  double direct_finite_return_ratio_{0.0};
  double diagnostic_finite_return_ratio_{0.0};
  double imu_roll_{0.0};
  double imu_pitch_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  double latest_x_{0.0};
  double latest_y_{0.0};
  double latest_yaw_{0.0};
  double delta_translation_m_{0.0};
  double delta_yaw_rad_{0.0};
  double translation_speed_mps_{0.0};
  double yaw_rate_radps_{0.0};
  std::string odom_reason_{"WAITING_FOR_ODOM"};
  std::string state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomMonitor>());
  rclcpp::shutdown();
  return 0;
}
