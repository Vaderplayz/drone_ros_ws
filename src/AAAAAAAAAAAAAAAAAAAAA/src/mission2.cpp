#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;


void publish_takeoff_setpoint(
       rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr publisher)
    {
    mavros_msgs::msg::PositionTarget setpoint_msg;
    setpoint_msg.header.stamp = node->get_clock()->now();
    setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                             mavros_msgs::msg::PositionTarget::IGNORE_VY |
                             mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    setpoint_msg.position.x = 0;
    setpoint_msg.position.y = 0;
    setpoint_msg.position.z = 5;
    setpoint_msg.yaw = 0;
    publisher->publish(setpoint_msg);
    }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // 2. Create a simple node. The constructor should be lightweight.
    auto node = std::make_shared<rclcpp::Node>("offboard_control");
    // 3. Create the service client using the node.
    auto client1 = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto setpoint_pub = node->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
  
  
    rclcpp::Rate rate(10); // 10 Hz
    //setpoin stream
RCLCPP_INFO(node->get_logger(), "Streaming setpoints to prime the connection...");
    for (int i = 0; i < 10; ++i) {
        publish_takeoff_setpoint(node, setpoint_pub);
        rclcpp::spin_some(node);
        rate.sleep();
    }


//set off board
      RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(request);
    }
//     // 6. Spin the node until the future is complete. This is the key step.
if (rclcpp::spin_until_future_complete(node, future_result1) == rclcpp::FutureReturnCode::SUCCESS) {
    if (future_result1.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Drone OFFBOARD successfully!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to OFOARD drone.");
    }
} else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call set mode service.");
}



//arming
    // 5. Create the request and send it asynchronously. This returns a "future".
    auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
arming_request->value = true; // true to arm, false to disarm

auto arming_future = arming_client->async_send_request(arming_request);

// Spin and wait for the arming result
if (rclcpp::spin_until_future_complete(node, arming_future) == rclcpp::FutureReturnCode::SUCCESS) {
    if (arming_future.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Drone ARMED successfully!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to arm drone.");
    }
} else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call arming service.");
}

    for (int i = 0; i < 10; ++i) {
        publish_takeoff_setpoint(node, setpoint_pub);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 8. Shut down ROS 2 cleanly.
    // rclcpp::shutdown();
    return 0;
}

