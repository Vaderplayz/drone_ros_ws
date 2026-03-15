
    #include "rclcpp/rclcpp.hpp"
    #include "mavros_msgs/srv/set_mode.hpp"
    #include "mavros_msgs/srv/command_bool.hpp"
    #include "mavros_msgs/msg/position_target.hpp"
    #include "sensor_msgs/msg/nav_sat_fix.hpp"
    #include "mavros_msgs/srv/waypoint_clear.hpp"
    #include "mavros_msgs/srv/waypoint_push.hpp"
    #include "mavros_msgs/msg/waypoint.hpp"
    #include <memory>
    #include <chrono>
    #include <functional>
    #include <cmath>

    using namespace std::chrono_literals;


    class OffboardControl : public rclcpp::Node {
    public:
        OffboardControl() : Node("offboard_control_mavros"),
            qos(rclcpp::SensorDataQoS()),
            R(50.0f),
            omega(0.2f),
            reachedHeight(false),
            setOffboard_(false),
            desire_height_(20),
            altitude(0.0f),
            current_latitude_(0.0f),
            current_longitude_(0.0f),
            home_altitude_(0.0f),
            target_altitude_(0.0f)
        {
            start_time = this->now();
            client_clear = this->create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
            client_push = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
            client1_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
            arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
            setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
            subscription_pose_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/mavros/global_position/global",
                qos,
                std::bind(&OffboardControl::callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");
            std::this_thread::sleep_for(3s);
            waitForPosition();
            pushMission();
            std::this_thread::sleep_for(3s);
            setAUTO();
            arm();
            timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_takeoff_setpoint, this));
            offboard_timer_ = this->create_wall_timer(100ms, [this]() {
                if (reachedHeight && !setOffboard_) {
                    setOffboard();
                    setOffboard_ = true;
                    reachedHeight = false;
                    offboard_timer_->cancel();
                }
            });
        }

    private:
        // ROS 2 interfaces
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;
        rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr client_push;
        rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr client_clear;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_pose_;
        rclcpp::TimerBase::SharedPtr offboard_timer_;
        rclcpp::QoS qos;
        rclcpp::Time start_time;

        // State variables
        float R;
        float omega;
        bool reachedHeight;
        float altitude;
        float current_latitude_;
        float current_longitude_;
        float home_altitude_;
        float target_altitude_;
        bool setOffboard_;
        int desire_height_;

        // Arm the drone
        void arm() {
            auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arming_request->value = true;
            while (!arming_client_->wait_for_service(1s) && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming service...");
            }
            auto arming_future = arming_client_->async_send_request(arming_request);
            // Optionally, handle the future result here
        }

        // Set the drone to OFFBOARD mode
        void setOffboard() {
            auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            set_mode_request->custom_mode = "OFFBOARD";
            while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
            }
            auto future_result = client1_->async_send_request(set_mode_request);
            // Optionally, handle the future result here
        }

        // Set the drone to AUTO.MISSION mode
        void setAUTO() {
            auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            set_mode_request->custom_mode = "AUTO.MISSION";
            while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
            }
            auto future_result = client1_->async_send_request(set_mode_request);
            // Optionally, handle the future result here
        }

        // Publish setpoints for takeoff and circular motion
        void publish_takeoff_setpoint() {
            double t = (this->now() - start_time).seconds();
            auto setpoint_msg = mavros_msgs::msg::PositionTarget();
            setpoint_msg.header.stamp = this->get_clock()->now();
            setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                     mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                     mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            if (setOffboard_) {
                // Circular motion
                setpoint_msg.velocity.x = omega * R * std::sin(omega * t);
                setpoint_msg.velocity.y = omega * R * std::cos(omega * t);
                setpoint_msg.velocity.z = 0;
                setpoint_msg.yaw = 0;
            } else {
                // Hold position
                setpoint_msg.velocity.x = 0;
                setpoint_msg.velocity.y = 0;
                setpoint_msg.velocity.z = 0;
                setpoint_msg.yaw = 0;
            }
            setpoint_pub_->publish(setpoint_msg);
        }

        // Push a simple mission (takeoff waypoint)
        void pushMission() {
            auto wp1 = mavros_msgs::msg::Waypoint();
            wp1.frame = 3;  // GLOBAL
            wp1.command = 22;  // NAV_TAKEOFF
            wp1.is_current = true;
            wp1.autocontinue = true;
            wp1.x_lat = current_latitude_;
            wp1.y_long = current_longitude_;
            wp1.z_alt = desire_height_;
            auto request_wp = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
            request_wp->waypoints.push_back(wp1);
            client_push->async_send_request(request_wp);
        }

        // GPS callback
        void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            altitude = msg->altitude;
            current_latitude_ = msg->latitude;
            current_longitude_ = msg->longitude;
            // Check if target height is reached and we haven't already switched to offboard
            if (std::abs(altitude - target_altitude_) <= 1.0 && !setOffboard_) {
                reachedHeight = true;
                RCLCPP_INFO(this->get_logger(), "Target height reached! Altitude: %.2f, Target: %.2f", altitude, target_altitude_);
            }
            RCLCPP_INFO(this->get_logger(), "Altitude: %.2f, Target: %.2f, ReachedHeight: %s, SetOffboard: %s",
                altitude, target_altitude_, reachedHeight ? "true" : "false", setOffboard_ ? "true" : "false");
        }

        // Wait for a valid GPS fix before starting
        void waitForPosition() {
            RCLCPP_INFO(this->get_logger(), "Waiting for valid GPS fix...");
            rclcpp::Rate rate(1.0);
            while (rclcpp::ok() && (current_latitude_ == 0.0 || current_longitude_ == 0.0)) {
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "Received GPS fix: Lat = %.8f, Lon = %.8f", current_latitude_, current_longitude_);
            home_altitude_ = altitude;
            target_altitude_ = home_altitude_ + desire_height_;
        }
    };

    int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OffboardControl>());
        rclcpp::shutdown();
        return 0;
    }


