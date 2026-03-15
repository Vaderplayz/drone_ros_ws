#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;
}

class OdomGpsValidatorNode : public rclcpp::Node {
public:
  OdomGpsValidatorNode() : Node("odom_gps_validator") {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "/mavros/global_position/global");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    path_odom_topic_ = declare_parameter<std::string>("path_odom_topic", "/validation/path_odom");
    path_gps_topic_ =
        declare_parameter<std::string>("path_gps_topic", "/validation/path_gps_local");
    report_period_sec_ = declare_parameter<double>("report_period_sec", 2.0);
    max_pair_age_sec_ = declare_parameter<double>("max_pair_age_sec", 0.5);
    max_path_points_ = declare_parameter<int>("max_path_points", 5000);
    csv_path_ = declare_parameter<std::string>("csv_path", "");

    auto qos_sensor = rclcpp::SensorDataQoS();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos_sensor,
        std::bind(&OdomGpsValidatorNode::odomCallback, this, std::placeholders::_1));
    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, qos_sensor,
        std::bind(&OdomGpsValidatorNode::gpsCallback, this, std::placeholders::_1));

    pub_path_odom_ = create_publisher<nav_msgs::msg::Path>(path_odom_topic_, 10);
    pub_path_gps_ = create_publisher<nav_msgs::msg::Path>(path_gps_topic_, 10);

    path_odom_.header.frame_id = odom_frame_;
    path_gps_.header.frame_id = odom_frame_;

    const auto report_period = std::chrono::duration<double>(std::max(0.2, report_period_sec_));
    report_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(report_period),
        std::bind(&OdomGpsValidatorNode::reportStats, this));

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_.is_open()) {
        RCLCPP_WARN(get_logger(), "Failed to open csv_path='%s'", csv_path_.c_str());
      } else {
        csv_ << "stamp_sec,odom_x,odom_y,gps_x,gps_y,error_xy\n";
      }
    }

    RCLCPP_INFO(get_logger(),
                "odom_gps_validator started. odom='%s' gps='%s' frame='%s' "
                "pair_age<=%.2fs",
                odom_topic_.c_str(), gps_topic_.c_str(), odom_frame_.c_str(),
                std::max(0.05, max_pair_age_sec_));
  }

private:
  struct XYPoint {
    double x{0.0};
    double y{0.0};
  };

  std::optional<XYPoint> projectGpsToLocal(double lat_deg, double lon_deg) {
    if (!gps_origin_) {
      gps_origin_ = sensor_msgs::msg::NavSatFix();
      gps_origin_->latitude = lat_deg;
      gps_origin_->longitude = lon_deg;
      gps_origin_->altitude = 0.0;
      RCLCPP_INFO(get_logger(), "GPS origin locked: lat=%.8f lon=%.8f", lat_deg, lon_deg);
    }

    const double lat0 = gps_origin_->latitude * kDegToRad;
    const double dlat = (lat_deg - gps_origin_->latitude) * kDegToRad;
    const double dlon = (lon_deg - gps_origin_->longitude) * kDegToRad;

    XYPoint p;
    p.x = dlon * kEarthRadiusM * std::cos(lat0);  // east
    p.y = dlat * kEarthRadiusM;                   // north
    return p;
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
      return;
    }
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return;
    }

    auto local = projectGpsToLocal(msg->latitude, msg->longitude);
    if (!local) return;

    latest_gps_local_ = *local;
    latest_gps_stamp_ = msg->header.stamp;
    gps_ready_ = true;
  }

  void pushPathPoint(nav_msgs::msg::Path &path, const rclcpp::Time &stamp, double x, double y) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = odom_frame_;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);

    const std::size_t max_points = static_cast<std::size_t>(std::max(100, max_path_points_));
    while (path.poses.size() > max_points) {
      path.poses.erase(path.poses.begin());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const rclcpp::Time now_stamp(msg->header.stamp);
    const double ox = msg->pose.pose.position.x;
    const double oy = msg->pose.pose.position.y;

    if (!odom_origin_) {
      odom_origin_ = XYPoint{ox, oy};
      RCLCPP_INFO(get_logger(), "Odom origin locked: x=%.3f y=%.3f", ox, oy);
    }

    const double odom_x_rel = ox - odom_origin_->x;
    const double odom_y_rel = oy - odom_origin_->y;

    pushPathPoint(path_odom_, now_stamp, odom_x_rel, odom_y_rel);
    path_odom_.header.stamp = now_stamp;
    pub_path_odom_->publish(path_odom_);

    if (!gps_ready_) return;
    const rclcpp::Time gps_stamp(latest_gps_stamp_);
    const double age = std::abs((now_stamp - gps_stamp).seconds());
    if (age > std::max(0.05, max_pair_age_sec_)) {
      return;
    }

    pushPathPoint(path_gps_, now_stamp, latest_gps_local_.x, latest_gps_local_.y);
    path_gps_.header.stamp = now_stamp;
    pub_path_gps_->publish(path_gps_);

    const double ex = odom_x_rel - latest_gps_local_.x;
    const double ey = odom_y_rel - latest_gps_local_.y;
    const double err = std::hypot(ex, ey);

    ++paired_count_;
    sum_sq_err_ += err * err;
    if (err > max_err_) max_err_ = err;

    if (csv_.is_open()) {
      const double ts =
          static_cast<double>(msg->header.stamp.sec) +
          static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
      csv_ << std::fixed << std::setprecision(9) << ts << ","
           << std::setprecision(4) << odom_x_rel << "," << odom_y_rel << ","
           << latest_gps_local_.x << "," << latest_gps_local_.y << ","
           << err << "\n";
    }
  }

  void reportStats() {
    if (paired_count_ == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting for time-aligned odom+gps pairs...");
      return;
    }

    const double rmse = std::sqrt(sum_sq_err_ / static_cast<double>(paired_count_));
    RCLCPP_INFO(get_logger(),
                "Pairs=%zu RMSE_xy=%.3fm MaxErr_xy=%.3fm "
                "(odom_path=%zu gps_path=%zu)",
                paired_count_, rmse, max_err_, path_odom_.poses.size(), path_gps_.poses.size());
  }

  std::string odom_topic_;
  std::string gps_topic_;
  std::string odom_frame_;
  std::string path_odom_topic_;
  std::string path_gps_topic_;
  std::string csv_path_;
  double report_period_sec_{2.0};
  double max_pair_age_sec_{0.5};
  int max_path_points_{5000};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_gps_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::optional<XYPoint> odom_origin_;
  std::optional<sensor_msgs::msg::NavSatFix> gps_origin_;
  XYPoint latest_gps_local_{};
  builtin_interfaces::msg::Time latest_gps_stamp_{};
  bool gps_ready_{false};

  nav_msgs::msg::Path path_odom_;
  nav_msgs::msg::Path path_gps_;

  std::size_t paired_count_{0};
  double sum_sq_err_{0.0};
  double max_err_{0.0};

  std::ofstream csv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomGpsValidatorNode>());
  rclcpp::shutdown();
  return 0;
}
