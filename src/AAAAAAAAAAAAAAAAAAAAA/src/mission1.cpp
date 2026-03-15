#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

// MAVROS-specific message and service types
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

// A simple state machine for our node
enum class NodeState {
    WAITING_FOR_CONNECTION,
    // WAITING_FOR_GUIDED, // MAVROS uses "GUIDED" for offboard readiness
    STREAMING_SETPOINTS,
    REQUESTING_OFFBOARD,
    REQUESTING_ARM,
    IN_CONTROL
};

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control_mavros") {
        // Quality of Service profile for state subscription
        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        // Publishers, Subscribers, and Service Clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", state_qos, std::bind(&OffboardControl::state_cb, this, _1));
        
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");

        // Timer to run the main control logic
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControl::control_loop, this));
        last_request_time_ = this->get_clock()->now();

    }

private:
    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    mavros_msgs::msg::State current_state_;
    rclcpp::Time last_request_time_;
    NodeState node_state_ = NodeState::WAITING_FOR_CONNECTION;

    // Callback for MAVROS state
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
        if (current_state_.connected && node_state_ == NodeState::WAITING_FOR_CONNECTION) {
            RCLCPP_INFO(this->get_logger(), "MAVROS connected! Waiting for OFFBOARD mode to be available.");
            node_state_ = NodeState::REQUESTING_OFFBOARD;
        }
    }

    // Main control loop for the state machine
    void control_loop() {
        // Always stream setpoints, regardless of state, to satisfy PX4's requirement
        publish_takeoff_setpoint();

        switch (node_state_) {
            case NodeState::WAITING_FOR_CONNECTION:
                // Do nothing, wait for state_cb to advance state
                break;

            // case NodeState::WAITING_FOR_GUIDED:
            //     if (current_state_.guided) {
            //         RCLCPP_INFO(this->get_logger(), "GUIDED mode is available. Requesting OFFBOARD mode.");
            //         node_state_ = NodeState::RE;
            //         last_request_time_ = this->get_clock()->now();
            //     }
            //     break;

            case NodeState::REQUESTING_OFFBOARD:
                if (current_state_.mode != "OFFBOARD") {
                    if ((this->get_clock()->now() - last_request_time_) > 2s) {
                        request_offboard_mode();
                        last_request_time_ = this->get_clock()->now();
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Switched to OFFBOARD mode. Requesting ARM.");
                    node_state_ = NodeState::REQUESTING_ARM;
                }
                break;

            case NodeState::REQUESTING_ARM:
                if (!current_state_.armed) {
                    if ((this->get_clock()->now() - last_request_time_) > 2s) {
                        request_arm();
                        last_request_time_ = this->get_clock()->now();
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Vehicle is ARMED. Offboard control is active.");
                    node_state_ = NodeState::IN_CONTROL;
                }
                break;

            case NodeState::IN_CONTROL:
                // The drone is now armed and in offboard mode.
                // The timer will continue to publish setpoints.
                // You can add more complex logic here to fly a mission.
                break;
        }
    }

    // Publishes a setpoint for taking off to 5 meters
    void publish_takeoff_setpoint() {
        auto setpoint_msg = mavros_msgs::msg::PositionTarget();
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
        setpoint_msg.position.z = 5; // Target altitude of 5 meters (in MAVROS ENU frame, +Z is up)
        setpoint_msg.yaw = 0;
        setpoint_pub_->publish(setpoint_msg);
    }

    // Sends a request to switch to OFFBOARD mode
    void request_offboard_mode() {
        RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(request);
    }

    // Sends a request to arm the vehicle
    void request_arm() {
        RCLCPP_INFO(this->get_logger(), "Requesting ARM...");
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arming_client_->async_send_request(request);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
