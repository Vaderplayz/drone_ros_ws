
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <memory>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class MissionControl : public rclcpp::Node {
public:
    MissionControl() : Node("mission_control_mavros") {

        client1_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        client_clear = this->create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
        client_push = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
        subscription_pose_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global",
            qos,
            std::bind(&MissionControl::callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");
        
        // Create timer for periodic checks
        timer_ = this->create_wall_timer(
            5000ms,
            std::bind(&MissionControl::timer_callback, this)
        );

    waitForPosition();
    pushMission();

    setAUTO();
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    arm();
    if (reachedWaypoint){    
        setRTL();
};
        


}
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_pose_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr client_push;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr client_clear;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    bool reachedWaypoint = false;
    mavros_msgs::msg::Waypoint wp2_;


    void arm();
    void setAUTO();
    void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void waitForPosition();
    void pushMission();

    void setRTL();
    void timer_callback();
    double current_latitude_;
    double current_longitude_;
    double home_latitude_;
    double home_longitude_;

};



void MissionControl::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Latitude: %.8f, Longitude: %.8f", current_latitude_, current_longitude_);
}

void MissionControl::callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_latitude_ = msg->latitude;
        current_longitude_ = msg->longitude;
        double altitude = msg->altitude;
   if (std::abs(current_latitude_ - wp2_.x_lat) <= 0.0000003 &&
            std::abs(current_longitude_ - wp2_.y_long) <= 0.0000003) {
            reachedWaypoint = true;

        }
    home_latitude_ = current_latitude_;
    home_longitude_ = current_longitude_;
    RCLCPP_INFO(this->get_logger(), "Lat: %.6f, Lon: %.6f, Alt: %.2f, Home%.6f, %.6f",current_latitude_, current_longitude_, altitude, home_latitude_, home_longitude_);

}; 


void MissionControl::arm(){
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


void MissionControl::setAUTO(){
        auto request1 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request1->custom_mode = "AUTO.MISSION";
        auto future_result1 = client1_->async_send_request(request1);
        while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
}

    // 6. Spin the Node until the future is complete. This is the key step.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result1) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future_result1.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Drone set AUTO successfully!");
        }   else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode.");}}}


void MissionControl::pushMission(){
// std::vector<mavros_msgs::msg::Waypoint> wp_list;
    auto clear_req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
        // Waypoint 1: Takeoff
        mavros_msgs::msg::Waypoint wp1;
        wp1.frame = 3;  // GLOBAL
        wp1.command = 22;  // NAV_TAKEOFF
        wp1.is_current = true;
        wp1.autocontinue = true;
        wp1.x_lat = current_latitude_;
        wp1.y_long = current_longitude_;
        wp1.z_alt = 10.0;

        // Waypoint 2: Move to point
        mavros_msgs::msg::Waypoint wp2;
        wp2.frame = 3;  // GLOBAL
        wp2.command = 16;  // NAV_WAYPOINT
        wp2.is_current = false;
        wp2.autocontinue = true;
        wp2.x_lat = 47.398156;
        wp2.y_long = 8.543653;
        wp2.z_alt = 10.0;

        wp2_ = wp2;


        //rtl
        mavros_msgs::msg::Waypoint rtl_wp;
        rtl_wp.frame = 3;  // GLOBAL
        rtl_wp.command = 16;  // NAV_WAYPOINT
        rtl_wp.is_current = false;
        rtl_wp.autocontinue = true;
        rtl_wp.x_lat = home_latitude_;
        rtl_wp.y_long = home_longitude_;
        rtl_wp.z_alt = 1.0;

 

        //land
        mavros_msgs::msg::Waypoint land;
        land.command = 21;  // NAV_WAYPOINT
        land.x_lat = home_latitude_;
        land.y_long = home_longitude_;
        land.z_alt = 0.0;

        auto request_wp = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
        request_wp->waypoints.push_back(wp1);
        request_wp->waypoints.push_back(wp2);
        request_wp->waypoints.push_back(rtl_wp);
        request_wp->waypoints.push_back(land);


        auto future_push = client_push->async_send_request(request_wp);

}

void MissionControl::setRTL(){
        auto request2 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request2->custom_mode = "RTL" ;       
            auto future_result2 = client1_->async_send_request(request2);
        while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
}

    // 6. Spin the Node until the future is complete. This is the key step.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result2) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future_result2.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Drone set RTL successfully!");
        }   else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode.");}}}


void MissionControl::waitForPosition() {
    RCLCPP_INFO(this->get_logger(), "Waiting for valid GPS fix...");
    rclcpp::Rate rate(1.0);  // 1 Hz
    while (rclcpp::ok() && (current_latitude_ == 0.0 || current_longitude_ == 0.0)) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Received GPS fix: Lat = %.8f, Lon = %.8f", current_latitude_, current_longitude_);

}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControl>());
    rclcpp::shutdown();
    return 0;
}

