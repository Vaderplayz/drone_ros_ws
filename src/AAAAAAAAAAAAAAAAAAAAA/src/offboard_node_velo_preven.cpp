#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <chrono>
#include <thread>
#include <string>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control") {
        // QoS for local position        
        // Publisher for initial hover setpoint
        pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        // Publisher for goal setpoint (to path planner)
        goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/drone_goal", 10);

        // Subscriber for local position
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&OffboardControl::local_position_cb, this, std::placeholders::_1));

        // Arm and set mode clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        RCLCPP_INFO(this->get_logger(), "Offboard Control Node started.");
        
        // Allow PX4 to initialize subscribers
        std::this_thread::sleep_for(2s);

        // Arm and switch to offboard
        arm_and_set_mode();

        // Publish initial hover setpoint (only once)
        publish_initial_hover();

        // After initial hover, ask user for goal
        ask_user_for_goal();
    }

private:
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    geometry_msgs::msg::PoseStamped current_pose_;

    void local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
    }

    void arm_and_set_mode() {
        // Wait for services
        arming_client_->wait_for_service();
        set_mode_client_->wait_for_service();

        // Arm
        auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_req->value = true;
        auto arm_res = arming_client_->async_send_request(arm_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_res) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Vehicle armed.");
        }

        // Set mode
        auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        mode_req->custom_mode = "OFFBOARD";
        auto mode_res = set_mode_client_->async_send_request(mode_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_res) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Offboard mode set.");
        }
    }

    void publish_initial_hover() {
        geometry_msgs::msg::PoseStamped hover;
        hover.header.stamp = this->get_clock()->now();
        hover.pose.position.x = current_pose_.pose.position.x;
        hover.pose.position.y = current_pose_.pose.position.y;
        hover.pose.position.z = 2.0;  // Fixed altitude = 2m

        // Send a few times to lock offboard
        for (int i = 0; i < 20; i++) {
            pos_pub_->publish(hover);
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Initial hover setpoint published.");
    }

    void ask_user_for_goal() {
        double gx, gy, gz;
        std::cout << "Enter goal x y z: ";
        std::cin >> gx >> gy >> gz;

        geometry_msgs::msg::Point goal;
        goal.x = gx;
        goal.y = gy;
        goal.z = gz;

        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published user goal: [%.2f, %.2f, %.2f]", gx, gy, gz);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
