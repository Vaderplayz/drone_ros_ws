#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>
#include <string>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control_mavros") {
        client1_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // Subscribe to local position
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&OffboardControl::local_position_cb, this, std::placeholders::_1));

        // Subscribe to lidar scan
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan", qos,
            std::bind(&OffboardControl::lidar_cb, this, std::placeholders::_1));

        // Timer for publishing setpoints
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_setpoint, this));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");

        std::this_thread::sleep_for(1s);
        setOffboard();
        std::this_thread::sleep_for(3s);

        arm();

        // Launch input thread
        input_thread_ = std::thread(&OffboardControl::input_loop, this);
        input_thread_.detach();
    }

private:
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    std::mutex mtx_;
    double x_{0}, y_{0}, z_{2};
    double current_x_{0}, current_y_{0};
    bool got_initial_pos_{false};
    bool front_blocked_{false};
    std::thread input_thread_;

    double yaw_setpoint_{0.0};
    bool yaw_set_{false};

    // === Core Functions ===
    void arm();
    void setOffboard();
    void setMode(const std::string &mode);
    void publish_setpoint();
    void input_loop();
    void local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void lidar_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    double compute_yaw(double target_x, double target_y);
};

void OffboardControl::arm() {
    auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arming_request->value = true;

    while (!arming_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming service...");
    }

    auto future = arming_client_->async_send_request(arming_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        if (future.get()->success)
            RCLCPP_INFO(this->get_logger(), "Drone ARMED successfully!");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone.");
    }
}

void OffboardControl::setOffboard() {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
    }

    auto future = client1_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        if (future.get()->mode_sent)
            RCLCPP_INFO(this->get_logger(), "Drone set OFFBOARD successfully!");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode.");
    }
}

void OffboardControl::setMode(const std::string &mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    auto future = client1_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        if (future.get()->mode_sent)
            RCLCPP_INFO(this->get_logger(), "Mode set to %s", mode.c_str());
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode %s", mode.c_str());
    }
}

void OffboardControl::publish_setpoint() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_initial_pos_) return;

    auto msg = mavros_msgs::msg::PositionTarget();
    msg.header.stamp = this->get_clock()->now();
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    if (yaw_set_) {
        msg.yaw = yaw_setpoint_;
    } else {
        msg.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    }

    if (front_blocked_) {
        msg.position.x = current_x_;
        msg.position.y = current_y_;
        msg.position.z = z_;
        RCLCPP_WARN(this->get_logger(), "Front blocked! Holding position..., need a new setpoint:");
    } else {
        msg.position.x = x_;
        msg.position.y = y_;
        msg.position.z = z_;
    }

    setpoint_pub_->publish(msg);
}

void OffboardControl::local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;

    if (!got_initial_pos_) {
        x_ = current_x_;
        y_ = current_y_;
        z_ = 2.0;
        got_initial_pos_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial position locked at (%.2f, %.2f), target altitude %.2f",
                    x_, y_, z_);
    }
}

void OffboardControl::lidar_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int n = msg->ranges.size();
    double angle_min = msg->angle_min;
    double angle_inc = msg->angle_increment;

    double half_angle = 5.0 * M_PI / 180.0;
    double min_front = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; i++) {
        double angle = angle_min + i * angle_inc;
        if (fabs(angle) < half_angle) {
            if (msg->ranges[i] < min_front) {
                min_front = msg->ranges[i];
            }
        }
    }

    front_blocked_ = (min_front < 5.0);
}

double OffboardControl::compute_yaw(double target_x, double target_y) {
    double dx = target_x - current_x_;
    double dy = target_y - current_y_;
    return atan2(dy, dx);
}

void OffboardControl::input_loop() {
    while (rclcpp::ok()) {
        std::string input;
        std::cout << "Enter setpoint (x y z) or command (land/RTL): ";
        std::getline(std::cin, input);

        if (input == "land") {
            RCLCPP_INFO(this->get_logger(), "Landing...");
            setMode("AUTO.LAND");
        } else if (input == "RTL") {
            RCLCPP_INFO(this->get_logger(), "Returning to launch...");
            setMode("AUTO.RTL");
        } else {
            double x, y, z;
            if (sscanf(input.c_str(), "%lf %lf %lf", &x, &y, &z) == 3) {
                std::lock_guard<std::mutex> lock(mtx_);
                x_ = x;
                y_ = y;
                z_ = z;
                yaw_setpoint_ = compute_yaw(x_, y_);
                yaw_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Updated setpoint -> x=%.2f y=%.2f z=%.2f (yaw=%.2f rad)",
                            x_, y_, z_, yaw_setpoint_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid input.");
            }
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
