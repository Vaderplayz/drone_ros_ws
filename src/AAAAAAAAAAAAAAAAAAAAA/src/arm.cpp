#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class ArmDrone : public rclcpp::Node {
public:
    ArmDrone() : Node("arm") {
        client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;  // Arm

        auto result = client_->async_send_request(request);
        result.wait();

        if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Drone armed!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
        }
    }

private:
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmDrone>());
    rclcpp::shutdown();
    return 0;
}

