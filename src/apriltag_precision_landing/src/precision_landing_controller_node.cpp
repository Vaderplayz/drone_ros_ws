#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
}  // namespace

class PrecisionLandingControllerNode : public rclcpp::Node {
 public:
  PrecisionLandingControllerNode() : Node("precision_landing_controller") {
    drone_pose_topic_ = declare_parameter<std::string>("drone_pose_topic", "/mavros/local_position/pose");
    landing_target_topic_ = declare_parameter<std::string>("landing_target_topic", "/mavros/landing_target/pose");
    mavros_state_topic_ = declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel");
    command_frame_ = declare_parameter<std::string>("command_frame", "map");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    heartbeat_when_idle_ = declare_parameter<bool>("heartbeat_when_idle", true);

    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 0.40);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.35);
    require_offboard_mode_ = declare_parameter<bool>("require_offboard_mode", true);

    kp_xy_ = declare_parameter<double>("kp_xy", 0.65);
    kp_z_ = declare_parameter<double>("kp_z", 0.85);
    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.60);
    max_z_up_speed_ = declare_parameter<double>("max_z_up_speed", 0.40);
    max_z_down_speed_ = declare_parameter<double>("max_z_down_speed", 0.45);

    approach_height_m_ = declare_parameter<double>("approach_height_m", 1.8);
    enter_descend_xy_tol_m_ = declare_parameter<double>("enter_descend_xy_tol_m", 0.20);
    enter_descend_height_tol_m_ = declare_parameter<double>("enter_descend_height_tol_m", 0.35);
    break_descend_xy_tol_m_ = declare_parameter<double>("break_descend_xy_tol_m", 0.35);

    descent_rate_mps_ = declare_parameter<double>("descent_rate_mps", 0.22);
    final_xy_tol_m_ = declare_parameter<double>("final_xy_tol_m", 0.12);
    land_trigger_height_m_ = declare_parameter<double>("land_trigger_height_m", 0.55);
    stable_before_land_sec_ = declare_parameter<double>("stable_before_land_sec", 1.0);

    lost_target_behavior_ = declare_parameter<std::string>("lost_target_behavior", "hover");
    auto_land_mode_ = declare_parameter<std::string>("auto_land_mode", "AUTO.LAND");
    enable_auto_land_request_ = declare_parameter<bool>("enable_auto_land_request", true);
    mode_request_retry_sec_ = declare_parameter<double>("mode_request_retry_sec", 1.0);

    const auto qos_sensor = rclcpp::SensorDataQoS();
    const auto qos_cmd = rclcpp::QoS(10).reliable();

    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        drone_pose_topic_, qos_sensor,
        std::bind(&PrecisionLandingControllerNode::dronePoseCb, this, std::placeholders::_1));

    sub_landing_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        landing_target_topic_, qos_sensor,
        std::bind(&PrecisionLandingControllerNode::landingTargetCb, this, std::placeholders::_1));

    sub_state_ = create_subscription<mavros_msgs::msg::State>(
        mavros_state_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&PrecisionLandingControllerNode::stateCb, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, qos_cmd);
    pub_state_text_ = create_publisher<std_msgs::msg::String>("/precision_landing/state", 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&PrecisionLandingControllerNode::loop, this));

    RCLCPP_INFO(get_logger(),
                "precision_landing_controller started: pose=%s target=%s cmd=%s require_offboard=%s",
                drone_pose_topic_.c_str(), landing_target_topic_.c_str(), cmd_vel_topic_.c_str(),
                require_offboard_mode_ ? "true" : "false");
  }

 private:
  enum class Phase {
    WAIT_INPUT,
    WAIT_OFFBOARD,
    APPROACH,
    DESCEND,
    AUTO_LAND,
  };

  static const char *phaseName(Phase p) {
    switch (p) {
      case Phase::WAIT_INPUT: return "WAIT_INPUT";
      case Phase::WAIT_OFFBOARD: return "WAIT_OFFBOARD";
      case Phase::APPROACH: return "APPROACH";
      case Phase::DESCEND: return "DESCEND";
      case Phase::AUTO_LAND: return "AUTO_LAND";
      default: return "UNKNOWN";
    }
  }

  bool stale(const rclcpp::Time &stamp, double timeout_sec) const {
    return (now() - stamp).seconds() > timeout_sec;
  }

  void dronePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    drone_pose_ = *msg;
    got_drone_pose_ = true;
    last_pose_time_ = now();
  }

  void landingTargetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    landing_target_pose_ = *msg;
    got_target_pose_ = true;
    last_target_time_ = now();
  }

  void stateCb(const mavros_msgs::msg::State::SharedPtr msg) {
    mavros_state_ = *msg;
    got_mavros_state_ = true;
  }

  void publishCmd(double vx, double vy, double vz, double wz) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = command_frame_;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.z = wz;
    pub_cmd_->publish(cmd);
  }

  void publishDebugState(Phase phase, double dx, double dy, double alt) {
    std::ostringstream oss;
    oss << phaseName(phase)
        << " mode=" << (got_mavros_state_ ? mavros_state_.mode : "?")
        << " armed=" << (got_mavros_state_ && mavros_state_.armed ? "1" : "0")
        << " dx=" << std::fixed << std::setprecision(2) << dx
        << " dy=" << dy
        << " alt_above_tag=" << alt;

    std_msgs::msg::String msg;
    msg.data = oss.str();
    pub_state_text_->publish(msg);
  }

  void requestAutoLand() {
    if (!enable_auto_land_request_ || auto_land_requested_ || auto_land_request_pending_) {
      return;
    }

    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting /mavros/set_mode service for AUTO.LAND...");
      return;
    }

    const rclcpp::Time now_t = now();
    if ((now_t - last_mode_request_time_).seconds() < mode_request_retry_sec_) {
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = auto_land_mode_;

    auto_land_request_pending_ = true;
    last_mode_request_time_ = now_t;

    auto cb = [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
      bool ok = false;
      try {
        const auto resp = fut.get();
        ok = (resp && resp->mode_sent);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "AUTO.LAND request exception: %s", e.what());
      }

      auto_land_request_pending_ = false;
      if (ok) {
        auto_land_requested_ = true;
        RCLCPP_INFO(get_logger(), "AUTO.LAND request sent.");
      } else {
        RCLCPP_WARN(get_logger(), "AUTO.LAND request rejected/failed.");
      }
    };

    set_mode_client_->async_send_request(req, cb);
  }

  void loop() {
    const bool pose_valid = got_drone_pose_ && !stale(last_pose_time_, pose_timeout_sec_);
    const bool target_valid = got_target_pose_ && !stale(last_target_time_, target_timeout_sec_);

    if (!pose_valid) {
      phase_ = Phase::WAIT_INPUT;
      if (heartbeat_when_idle_) publishCmd(0.0, 0.0, 0.0, 0.0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting valid drone pose.");
      publishDebugState(phase_, 0.0, 0.0, 0.0);
      return;
    }

    const bool auto_land_active = got_mavros_state_ && (mavros_state_.mode == auto_land_mode_);

    if (require_offboard_mode_ && !auto_land_active) {
      const bool offboard_ok = got_mavros_state_ && mavros_state_.mode == "OFFBOARD";
      if (!offboard_ok) {
        phase_ = Phase::WAIT_OFFBOARD;
        if (heartbeat_when_idle_) publishCmd(0.0, 0.0, 0.0, 0.0);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting OFFBOARD mode before precision landing control.");
        publishDebugState(phase_, 0.0, 0.0, 0.0);
        return;
      }
    }

    if (auto_land_active || phase_ == Phase::AUTO_LAND) {
      phase_ = Phase::AUTO_LAND;
      publishCmd(0.0, 0.0, 0.0, 0.0);
      publishDebugState(phase_, 0.0, 0.0, 0.0);
      return;
    }

    if (!target_valid) {
      phase_ = Phase::WAIT_INPUT;
      if (lost_target_behavior_ == "descend") {
        publishCmd(0.0, 0.0, -std::abs(descent_rate_mps_) * 0.3, 0.0);
      } else {
        publishCmd(0.0, 0.0, 0.0, 0.0);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                           "Landing target lost/stale. behavior=%s", lost_target_behavior_.c_str());
      publishDebugState(phase_, 0.0, 0.0, 0.0);
      return;
    }

    const double dx = landing_target_pose_.pose.position.x - drone_pose_.pose.position.x;
    const double dy = landing_target_pose_.pose.position.y - drone_pose_.pose.position.y;
    const double xy_dist = std::hypot(dx, dy);
    const double alt_above_tag = drone_pose_.pose.position.z - landing_target_pose_.pose.position.z;

    double vx = kp_xy_ * dx;
    double vy = kp_xy_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > max_xy_speed_ && vxy > 1e-6) {
      const double scale = max_xy_speed_ / vxy;
      vx *= scale;
      vy *= scale;
    }

    double vz = 0.0;

    if (phase_ == Phase::WAIT_INPUT || phase_ == Phase::WAIT_OFFBOARD || phase_ == Phase::APPROACH) {
      phase_ = Phase::APPROACH;
      const double z_err = approach_height_m_ - alt_above_tag;
      vz = clamp(kp_z_ * z_err, -max_z_down_speed_, max_z_up_speed_);

      if (xy_dist <= enter_descend_xy_tol_m_ && std::fabs(z_err) <= enter_descend_height_tol_m_) {
        phase_ = Phase::DESCEND;
        descend_stable_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    if (phase_ == Phase::DESCEND) {
      vz = -std::abs(descent_rate_mps_);

      if (xy_dist > break_descend_xy_tol_m_) {
        phase_ = Phase::APPROACH;
      } else {
        const bool final_xy_ok = xy_dist <= final_xy_tol_m_;
        const bool final_alt_ok = alt_above_tag <= land_trigger_height_m_;

        if (final_xy_ok && final_alt_ok) {
          if (descend_stable_start_.nanoseconds() == 0) {
            descend_stable_start_ = now();
          }
          const double stable_t = (now() - descend_stable_start_).seconds();
          if (stable_t >= stable_before_land_sec_) {
            requestAutoLand();
            phase_ = Phase::AUTO_LAND;
            vz = 0.0;
          }
        } else {
          descend_stable_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
      }
    }

    publishCmd(vx, vy, vz, 0.0);
    publishDebugState(phase_, dx, dy, alt_above_tag);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "phase=%s dx=%.2f dy=%.2f xy=%.2f alt=%.2f vz=%.2f",
                         phaseName(phase_), dx, dy, xy_dist, alt_above_tag, vz);
  }

  std::string drone_pose_topic_;
  std::string landing_target_topic_;
  std::string mavros_state_topic_;
  std::string cmd_vel_topic_;
  std::string command_frame_;

  double publish_rate_hz_{20.0};
  bool heartbeat_when_idle_{true};
  double pose_timeout_sec_{0.40};
  double target_timeout_sec_{0.35};
  bool require_offboard_mode_{true};

  double kp_xy_{0.65};
  double kp_z_{0.85};
  double max_xy_speed_{0.60};
  double max_z_up_speed_{0.40};
  double max_z_down_speed_{0.45};

  double approach_height_m_{1.8};
  double enter_descend_xy_tol_m_{0.20};
  double enter_descend_height_tol_m_{0.35};
  double break_descend_xy_tol_m_{0.35};

  double descent_rate_mps_{0.22};
  double final_xy_tol_m_{0.12};
  double land_trigger_height_m_{0.55};
  double stable_before_land_sec_{1.0};

  std::string lost_target_behavior_{"hover"};
  std::string auto_land_mode_{"AUTO.LAND"};
  bool enable_auto_land_request_{true};
  double mode_request_retry_sec_{1.0};

  Phase phase_{Phase::WAIT_INPUT};
  bool got_drone_pose_{false};
  bool got_target_pose_{false};
  bool got_mavros_state_{false};
  bool auto_land_requested_{false};
  bool auto_land_request_pending_{false};

  geometry_msgs::msg::PoseStamped drone_pose_;
  geometry_msgs::msg::PoseStamped landing_target_pose_;
  mavros_msgs::msg::State mavros_state_;

  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time descend_stable_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mode_request_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_landing_target_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_text_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionLandingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
