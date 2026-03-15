// offboard_prec_land.cpp
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <cmath>
#include <thread>
#include <array>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_square_flight") {
        client_mode_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        client_arm_  = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", qos);

        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1));

        // landing target (e.g. from apriltag node or mavros/vision)
        subscription_tf1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/landing_target/pose", qos,
            std::bind(&OffboardControl::target_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::states, this));
 
        // small delay to let publishers/subscribers spin up
        setOffboard();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (initialized_wp_){
        arm();
    }
    }

private:
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_mode_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_arm_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_tf1_;
    rclcpp::Time search_start_;

    std::vector<std::array<double,3>> waypoints_;
    size_t current_wp_ = 0;
    double speed_limit_ = 1.0;

            double init_x;
            double init_y;


    double current_x_ ;
    double current_y_ ;
    double current_z_;
    double current_yaw_;
    float target_alt_ = 3.0;
    double wp_tolerance_ = 0.2;

    // landing target (from landing_target/pose)
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;
    double Tag_roll, Tag_pitch, Tag_yaw;

    bool have_target_ = false;
    bool above_target = false;
    bool initialized_wp_ = false;

    bool searching_for_tag_ = false;

    void arm() {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        while (!client_arm_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming...");
        }
        auto fut = client_arm_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) == rclcpp::FutureReturnCode::SUCCESS) {
            if (fut.get()->success) RCLCPP_INFO(this->get_logger(), "Armed!");
            else RCLCPP_WARN(this->get_logger(), "Arm call returned false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arming service");
        }
    }

    void setOffboard() {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        while (!client_mode_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode...");
        }
        auto fut = client_mode_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) == rclcpp::FutureReturnCode::SUCCESS) {
            if (fut.get()->mode_sent) RCLCPP_INFO(this->get_logger(), "OFFBOARD set!");
            else RCLCPP_WARN(this->get_logger(), "SetMode returned mode_sent=false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call set_mode service");
        }
    }

    // landing-target callback (from tag detector)
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (current_wp_ < waypoints_.size()) {
        return;
    }
    // if(have_target_){
    //     return;
    // }
        target_x = msg->pose.position.x;
        target_y = msg->pose.position.y;
        target_z = msg->pose.position.z;
         tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    tf2::Matrix3x3(q).getRPY(Tag_roll, Tag_pitch, Tag_yaw);

        have_target_ = true;
        searching_for_tag_=false;
        // compute horizontal error and decide if we are "above" the target
        double ex = std::fabs(target_x - current_x_);
        double ey = std::fabs(target_y - current_y_);
        const double above_tol = 0.2; // horizontal tolerance to start descent

        above_target = (ex <= above_tol && ey <= above_tol);
        // Note: you can also include a minimum target confidence or z-check if needed
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
        current_z_ = msg->pose.position.z;
tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
        if (!initialized_wp_){
            init_x = msg->pose.position.x;
            init_y = msg->pose.position.y;
            build_square_waypoints();
            initialized_wp_ = true;
        }
        if (current_wp_ < waypoints_.size()) {
            auto &wp = waypoints_[current_wp_];
            double dx = wp[0] - current_x_;
            double dy = wp[1] - current_y_;
            double dz = wp[2] - current_z_;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < wp_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu (%.2f, %.2f, %.2f)",
                            current_wp_, wp[0], wp[1], wp[2]);
                current_wp_++;
            }
        }
        
    }

void build_square_waypoints() {
    waypoints_.clear();
    double d = 5.0; // half side length of square
    double alt = target_alt_;

    // Starting point
    waypoints_.push_back({init_x,       init_y,       alt});
    // Square around init point
    waypoints_.push_back({init_x + d,   init_y + d,   alt});
    waypoints_.push_back({init_x + d,   init_y - d,   alt});
    waypoints_.push_back({init_x - d,   init_y - d,   alt});
    waypoints_.push_back({init_x - d,   init_y + d,   alt});
}
    void states() {
        auto msg = mavros_msgs::msg::PositionTarget();
        msg.header.stamp = this->get_clock()->now();
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        if (!initialized_wp_) return; //check if wp valid?
    
        // If still in waypoint mission
        if (current_wp_ < waypoints_.size()) {
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ ;
            auto &wp = waypoints_[current_wp_];
double dx = wp[0] - current_x_;
double dy = wp[1] - current_y_;
double desired_yaw = std::atan2(dy, dx);

double yaw_error = std::atan2(std::sin(desired_yaw - current_yaw_),
                              std::cos(desired_yaw - current_yaw_));
double commanded_yaw = current_yaw_ + yaw_error;
            msg.position.x = wp[0];
            msg.position.y = wp[1];
            msg.position.z = wp[2];
            msg.yaw = commanded_yaw;
        } else {

            // vertical descend
            if (have_target_ && above_target) {
   
                msg.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ ;

                msg.velocity.x = 0.0;
                msg.velocity.y = 0.0;
                msg.velocity.z = -0.2;
                msg.yaw = Tag_yaw + M_PI / 2;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Above target: descending slowly...");

            } else if (have_target_ && !above_target ){
                // horizontal aproach
                msg.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW;

                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = current_z_;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Moving to hover above detected target at [%.2f, %.2f], have target = %s", target_x, target_y, have_target_ ? "true" : "false");
} else {
    // If no landing target seen, enter "search" mode
    if (!searching_for_tag_ && !have_target_) {
        searching_for_tag_ = true;
        search_start_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "No target detected, starting search...");
    }

    rclcpp::Duration elapsed = this->get_clock()->now() - search_start_;

    if (elapsed.seconds() < 10.0) {
        // Keep hovering at 5m while searching
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

msg.position.x = current_x_;   // hold current XY
msg.position.y = current_y_;
msg.position.z = 10.0;          // climb to 10m to search
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Searching for landing target (%.1f/10s)...", elapsed.seconds());
    } else {
        // Timeout reached → land
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_PZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        msg.velocity.x = 0.0;
        msg.velocity.y = 0.0;
        msg.velocity.z = -0.5;

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No target found after 10s → initiating landing...");
    }
}

            // hardlanding
            if (current_z_ <= 0.2) {
                msg.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

                msg.velocity.x = 0.0;
                msg.velocity.y = 0.0;
                msg.velocity.z = -3.0;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Hard landing at ground...");
            }
        }

        setpoint_pub_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
