#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/altitude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <chrono>
#include <functional>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

static double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control") {
        // clients & publisher
        client_set_mode_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        client_arming_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // subscriptions
        sub_alt_ = this->create_subscription<mavros_msgs::msg::Altitude>(
            "/mavros/altitude", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::altCallback, this, std::placeholders::_1));

        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting 1.5s for connections...");
        // Give some short time for topics/services to be ready
                timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_setpoint, this));

        std::this_thread::sleep_for(1500ms);

        // create periodic publisher (10 Hz)

        // enter OFFBOARD and arm (blocking until success or timeout)
        setOffboard();
        arm();

        RCLCPP_INFO(this->get_logger(), "Ready — waiting for initial pose to begin mission...");
    }

private:
    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_set_mode_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr sub_alt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;

    // pose & timing
    double init_x_ = NAN;
    double init_y_ = NAN;
    double cur_x_ = 0.0;
    double cur_y_ = 0.0;
    double cur_yaw_ = 0.0;
    double altitude_ = 0.0;   // relative altitude (m)
    rclcpp::Time yaw_hold_start_;

    // mission params
    const double target_alt_ = 2.0;    // takeoff altitude in meters
    const double forward_dist_ = 3.0;  // world X forward distance (m)
    const double yaw_hold_secs_ = 5.0; // hold after yaw (s)
    const double pos_tol_ = 0.25;      // position tolerance (m)
    const double max_speed_ = 1.0;     // used for fallback velocity (m/s)
    const double descend_speed_ = 0.6; // m/s descend

    // mission flags & computed values
    bool init_pose_received_ = false;
    bool reached_height_ = false;
    bool moved_forward_ = false;    // first forward
    bool yaw_started_ = false;      // yaw computed/started (so not continuous)
    bool yaw_done_ = false;         // completed hold
    bool moved_back_ = false;       // returned to home
    bool landing_started_ = false;
    bool disarmed_ = false;
    double target_yaw_ = 0.0;

    // utility: request arming
    void arm() {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        // wait for service
        if (!client_arming_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Arming service unavailable.");
            return;
        }
        auto fut = client_arming_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (fut.get()->success) RCLCPP_INFO(this->get_logger(), "Armed.");
            else RCLCPP_ERROR(this->get_logger(), "Arming request rejected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arming service call failed.");
        }
    }

    void disarm() {
        if (disarmed_) return;
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = false;
        if (!client_arming_->wait_for_service(3s)) {
            RCLCPP_WARN(this->get_logger(), "Disarm service unavailable.");
            return;
        }
        auto fut = client_arming_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (fut.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Disarmed.");
                disarmed_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Disarm rejected.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Disarm service call failed.");
        }
    }

    // utility: set OFFBOARD mode
    void setOffboard() {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        if (!client_set_mode_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "SetMode service not available.");
            return;
        }
        auto fut = client_set_mode_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (fut.get()->mode_sent) RCLCPP_INFO(this->get_logger(), "OFFBOARD set.");
            else RCLCPP_WARN(this->get_logger(), "OFFBOARD request not accepted.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "SetMode service call failed.");
        }
    }

    // main mission publisher
    void publish_setpoint() {
        auto msg = mavros_msgs::msg::PositionTarget();
        msg.header.stamp = this->get_clock()->now();
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // wait for initial pose to be received
        if (!init_pose_received_) {
            // gently climb a bit until we have pose
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_PX |
                mavros_msgs::msg::PositionTarget::IGNORE_PY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            msg.position.x = 0.0;
            msg.position.y = 0.0;
            msg.position.z = 0.5;
            msg.yaw = 0.0;
            setpoint_pub_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Waiting for initial local pose...");
            return;
        }

        // 1) Takeoff to target_alt_
        if (!reached_height_) {
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            // hold home XY and climb to target_alt_
            msg.position.x = init_x_;
            msg.position.y = init_y_;
            msg.position.z = target_alt_;
            msg.yaw = cur_yaw_; // keep current yaw until yaw stage
            setpoint_pub_->publish(msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Taking off: alt=%.2f / %.2f", altitude_, target_alt_);

            // when altitude reached (or safety timeout), mark reached
            if (altitude_ >= target_alt_ - 0.12) {
                reached_height_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached target altitude.");
                // small pause to stabilize
                yaw_hold_start_ = this->now();
            }
            return;
        }

        // 2) Move forward on world X by forward_dist_ (headless)
        if (reached_height_ && !moved_forward_) {
            double desired_x = init_x_ + forward_dist_;
            double desired_y = init_y_;
            // publish position target to desired X
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            msg.position.x = desired_x;
            msg.position.y = desired_y;
            msg.position.z = target_alt_;
            msg.yaw = cur_yaw_; // do not change yaw yet
            setpoint_pub_->publish(msg);

            double dx = cur_x_ - desired_x;
            double dy = cur_y_ - desired_y;
            double dist = std::sqrt(dx*dx + dy*dy);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Moving forward: dist_to_goal=%.2f m", dist);

            // success check
            if (dist <= pos_tol_) {
                moved_forward_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached forward waypoint.");
                // compute yaw target once
                target_yaw_ = normalize_angle(cur_yaw_ + M_PI);
                yaw_hold_start_ = this->now();
                yaw_started_ = true;
            }
            return;
        }

        // 3) Yaw 180 once and hold yaw_hold_secs_
        if (moved_forward_ && !yaw_done_) {
            // publish position hold at current spot with the computed target_yaw_
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            msg.position.x = cur_x_;
            msg.position.y = cur_y_;
            msg.position.z = target_alt_;
            msg.yaw = target_yaw_;
            setpoint_pub_->publish(msg);

            double elapsed = (this->now() - yaw_hold_start_).seconds();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Yaw hold: %.1f / %.1f s", elapsed, yaw_hold_secs_);
            if (elapsed >= yaw_hold_secs_) {
                yaw_done_ = true;
                RCLCPP_INFO(this->get_logger(), "Yaw hold finished; proceeding to return.");
            }
            return;
        }

        // 4) Move forward again (which returns home — since yaw flipped, forward in world X is back)
        if (yaw_done_ && !moved_back_) {
            double desired_x = init_x_; // go back home X
            double desired_y = init_y_;
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            msg.position.x = desired_x;
            msg.position.y = desired_y;
            msg.position.z = target_alt_;
            msg.yaw = target_yaw_;
            setpoint_pub_->publish(msg);

            double dx = cur_x_ - desired_x;
            double dy = cur_y_ - desired_y;
            double dist = std::sqrt(dx*dx + dy*dy);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Returning home: dist_to_home=%.2f m", dist);
            if (dist <= pos_tol_) {
                moved_back_ = true;
                RCLCPP_INFO(this->get_logger(), "Returned to home position.");
            }
            return;
        }

        // 5) Landing
        if (moved_back_ && !landing_started_) {
            landing_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting landing.");
        }

        if (landing_started_) {
            // if near ground, command zero and disarm
            if (altitude_ <= 0.15) {
                // send zero velocities / hold and disarm
                msg.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                msg.velocity.x = 0.0;
                msg.velocity.y = 0.0;
                msg.velocity.z = -5.0;
                setpoint_pub_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Landed (alt<=0.15). Disarming...");
                // disarm();
                // stop publishing further (but timer continues; will early return)
                return;
            } else {
                // descend
                msg.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                msg.velocity.x = 0.0;
                msg.velocity.y = 0.0;
                msg.velocity.z = -std::abs(descend_speed_);
                setpoint_pub_->publish(msg);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Landing... alt=%.2f", altitude_);
                return;
            }
        }

        // fallback: publish nominal hold at current pose
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        msg.position.x = cur_x_;
        msg.position.y = cur_y_;
        msg.position.z = std::max(0.2, altitude_);
        msg.yaw = cur_yaw_;
        setpoint_pub_->publish(msg);
    }

    // callbacks
    void altCallback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
        // msg->relative is height relative to takeoff (usually)
        altitude_ = std::abs(msg->relative);
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double px = msg->pose.position.x;
        double py = msg->pose.position.y;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        cur_x_ = px;
        cur_y_ = py;
        cur_yaw_ = yaw;

        if (!init_pose_received_) {
            init_pose_received_ = true;
            init_x_ = px;
            init_y_ = py;
            RCLCPP_INFO(this->get_logger(), "Initial position set: (%.2f, %.2f), yaw=%.2f",
                        init_x_, init_y_, cur_yaw_);
        }
    }
};
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardControl>();

    // Option A: Use executor (recommended for timers/callbacks)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // Option B: (simpler) just do:
    // rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}