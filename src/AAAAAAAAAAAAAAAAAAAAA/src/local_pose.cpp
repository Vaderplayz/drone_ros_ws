#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class GetPose : public rclcpp::Node {
public:
GetPose() : Node("Local_node_apr") {
subscription_tf_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose",
            qos,
                std::bind(&GetPose::callback, this, std::placeholders::_1)
            );
        }
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_tf_;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    float trans_x;
    float trans_y;
    float trans_z;
    float rot_x;
    float rot_y;
    float rot_z;
    float rot_w;
    double roll;
    double pitch;
    double yaw;

    void callback(const geometry_msgs::msg::PoseStamped msg);

};



void GetPose::callback(const geometry_msgs::msg::PoseStamped msg) {

     trans_x =  msg.pose.position.x;
     trans_y =  msg.pose.position.y;
     trans_z =  msg.pose.position.z;

     rot_x =  msg.pose.orientation.x;
     rot_y =  msg.pose.orientation.y;
     rot_z =  msg.pose.orientation.z;
     rot_w =  msg.pose.orientation.w;

    // Build tf2 quaternion from rotation
tf2::Quaternion q(rot_x, rot_y, rot_z, rot_w);

// Build tf2 vector from translation
tf2::Vector3 tr(trans_x, trans_y, trans_z);

tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
// double roll_deg  = roll  * 180.0 / M_PI;
// double pitch_deg = pitch * 180.0 / M_PI;
// double yaw_deg   = yaw   * 180.0 / M_PI;
    RCLCPP_INFO(this->get_logger(), " trans: %.2f, %.2f, %.2f, Roll %.2f,Pitch %.2f,  Yaw:  %.2f", 
    trans_x,trans_y,trans_z, roll, pitch, yaw);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetPose>());
    rclcpp::shutdown();
    return 0;
}



