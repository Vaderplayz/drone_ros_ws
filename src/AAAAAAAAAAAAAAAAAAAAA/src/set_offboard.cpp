#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
//input
int base_mode;
std::string flight_mode_custom;

std::string input_mode = argv[1];

if (std::isdigit(input_mode[0])) {
    base_mode = std::stoi(input_mode);
    flight_mode_custom = "";
}
else {
    flight_mode_custom = input_mode;
    base_mode = 0;  // Use 0 when setting custom mode
}
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);

    // 2. Create a simple node. The constructor should be lightweight.
    auto node = std::make_shared<rclcpp::Node>("arm_drone_client");

    // 3. Create the service client using the node.
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // 4. Wait for the service to be available.
 while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for /mavros/set_mode service...");
    }

    // 5. Create the request and send it asynchronously. This returns a "future".
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = flight_mode_custom ;
    request->base_mode = base_mode ;

    auto future_result = client->async_send_request(request);

    // 6. Spin the node until the future is complete. This is the key step.
    //    It processes ROS 2 events (like the service response) while waiting.
    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // 7. Check the result from the service call.
        if (future_result.get()->mode_sent) {
            RCLCPP_INFO(node->get_logger(), "Successfully!");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call setmode service.");
    }

    // 8. Shut down ROS 2 cleanly.
    rclcpp::shutdown();
    return 0;
}