#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/altitude.hpp"


#include <memory>
#include <chrono>
#include <functional>
#include <cmath>

using namespace std::chrono_literals;


class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control_mavros") {
        start_time = this->now();

        client1_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
        subscription_pose_ = this->create_subscription<mavros_msgs::msg::Altitude>(
            "/mavros/altitude",
            qos,
            std::bind(&OffboardControl::callback, this, std::placeholders::_1)
        );
    
   RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");
           // Start timer for periodic setpoint publishing
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_takeoff_setpoint, this));
    std::this_thread::sleep_for(std::chrono::seconds(3));   
    setOffboard();
    arm();
    if (limited_omega <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid omega detected! Cannot compute fly_time.");
    rclcpp::shutdown();
    return;
}
        


}
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr lap_timer_;
    rclcpp::TimerBase::SharedPtr land_timer_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;
    rclcpp::Time start_time;   // Time when motion starts}; 
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr subscription_pose_;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    void arm();
    void disarm();
    void setOffboard();
    void publish_takeoff_setpoint();
    void setAUTO();
    void callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
    bool reachedHeight = false;
    float altitude;
    float target = 5;
    double A = 2;
    double omega = 0.5; //rad/s; angular velocity
    float laps = 1;
    bool finished= false;
    double max_velocity = 1.5;  // m/s
    double limited_omega = std::min(omega, max_velocity / (A*sqrt(2)));
    int fly_time = 2*M_PI / limited_omega ;
    float vz = 1.5;
    int landing_time = target / vz;
    bool landing_mode_started = false;
    bool land_timer_started = false;
    bool already_disarmed = false;
    float vx = 0;
    float vy = 0;
}; 

void OffboardControl::arm(){
        auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arming_request->value = true; // true to arm, false to disarm
        auto arming_future = arming_client_->async_send_request(arming_request);
        while (!arming_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming service...");
}


    // Spin and wait for the arming result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arming_future) == rclcpp::FutureReturnCode::SUCCESS) {
        if (arming_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Drone ARMED successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone.");
    }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call arming service.");}}
        

void OffboardControl::setOffboard(){
        auto request1 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request1->custom_mode = "OFFBOARD" ;
        auto future_result1 = client1_->async_send_request(request1);
        while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
}

    // 6. Spin the Node until the future is complete. This is the key step.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result1) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future_result1.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Drone set OFFBOARD successfully!");
        }   else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode.");}}}


void OffboardControl::publish_takeoff_setpoint() {
            double t = (this->now() - start_time).seconds();
            auto setpoint_msg = mavros_msgs::msg::PositionTarget();
            setpoint_msg.header.stamp = this->get_clock()->now();
            setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            
vx = A*limited_omega*cos(limited_omega*t);
vy = A*limited_omega*cos(2*limited_omega*t);


            if (finished && altitude <= 0.2) {
                setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                setpoint_msg.velocity.x = 0;
                setpoint_msg.velocity.y = 0;
                setpoint_msg.velocity.z = -3;
            }
            else if (finished){
                setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_VY |                                        mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |                                        mavros_msgs::msg::PositionTarget::IGNORE_VY |

                                        
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                setpoint_msg.position.x = 0;
                setpoint_msg.position.y = 0;
                setpoint_msg.velocity.z = -0.2;

            }
            else if (reachedHeight) {
                // In offboard mode, publish velocity setpoints for circular motion
                setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                setpoint_msg.velocity.x = vx;
                setpoint_msg.velocity.y = vy;
                setpoint_msg.velocity.z = 0; 
            }
             else {
                // Before offboard mode, publish zero velocity to maintain position
                setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                setpoint_msg.velocity.x = 0;
                setpoint_msg.velocity.y = 0;
                setpoint_msg.velocity.z = vz; 
            }
            setpoint_pub_->publish(setpoint_msg);
    }



void OffboardControl::callback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
    altitude = abs(msg->relative);
    RCLCPP_INFO(this->get_logger(),"vx=: %.2f, vy: %.2f", vx, vy);
        RCLCPP_INFO(this->get_logger(), " Altitude: %.2f", altitude);



    if ((std::abs(altitude - target) <= 0.2 && !reachedHeight) || (altitude >=target&&!reachedHeight)) {
        reachedHeight = true;
        RCLCPP_INFO(this->get_logger(), "Target height reached! Altitude: %.2f, Target: %.2f", altitude, target);

        // Start the timer only once, when height is first reached
        lap_timer_ = this->create_wall_timer(
            std::chrono::seconds(fly_time),
            [this]() {
                finished = true;
                RCLCPP_INFO(this->get_logger(), "Fly time ended! finished = true");
                lap_timer_->cancel(); // Optional: stop the timer
            }
        );
        
    }


}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}


