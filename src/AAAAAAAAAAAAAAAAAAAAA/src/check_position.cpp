#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <unordered_map>


using namespace std::chrono_literals;

class GetPose : public rclcpp::Node {
public:
GetPose() : Node("Get_Pose_node_apr") {
subscription_tf1_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf",
            qos,
                std::bind(&GetPose::callbacktag, this, std::placeholders::_1)
            );

 initTags();  }
private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf1_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_tf2_;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    std::unordered_map<std::string, tf2::Transform> tag_world_poses;
void initTags(){
        tf2::Transform T0;
    T0.setOrigin(tf2::Vector3(0, 0, 0.001));
    tf2::Quaternion qt0;
    qt0.setRPY(0, 0, 0);   // roll=0, pitch=0, yaw=0
    T0.setRotation(qt0);
    tag_world_poses["id0"] = T0;
    
        tf2::Transform T1;
    T1.setOrigin(tf2::Vector3(5, 5, 0.001));
    tf2::Quaternion qt1;
    qt1.setRPY(0, 0, 0);   // roll=0, pitch=0, yaw=0
    T1.setRotation(qt1);
    tag_world_poses["id1"] = T1;

        tf2::Transform T2;
    T2.setOrigin(tf2::Vector3(5, -5, 0.001));
    tf2::Quaternion qt2;
    qt2.setRPY(0, 0, 0);   // roll=0, pitch=0, yaw=0
    T2.setRotation(qt2);
    tag_world_poses["id2"] = T2;

    tf2::Transform T3;
    T3.setOrigin(tf2::Vector3(-5, 5, 0.001));
    tf2::Quaternion qt3;
    qt3.setRPY(0, 0, 0);   // roll=0, pitch=0, yaw=0
    T3.setRotation(qt3);
    tag_world_poses["id3"] = T3;

        tf2::Transform T4;
    T4.setOrigin(tf2::Vector3(-5, -5, 0.001));
    tf2::Quaternion qt4;
    qt4.setRPY(0, 0, 0);   // roll=0, pitch=0, yaw=0
    T4.setRotation(qt4);
    tag_world_poses["id4"] = T4;
}

    std::string frame_child;
    std::string frame_parent;


    float trans_x1;
    float trans_y1;
    float trans_z1;

    float rot_x1;
    float rot_y1;
    float rot_z1;
    float rot_w1;

    double roll;
    double pitch;
    double yaw;


    tf2::Transform tf1;   // tag wrt camera
    tf2::Transform tf1_inv;   // cam wrt to tag
    tf2::Transform tf_wc;

    void callbacktag(const tf2_msgs::msg::TFMessage::SharedPtr msg); //get tag rela to cam
    void ComputeRealPose();

};


void GetPose::callbacktag(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
for (const auto &t : msg->transforms) {
    frame_child = t.child_frame_id;
    frame_parent = t.header.frame_id;

     trans_x1 =  t.transform.translation.x;
     trans_y1 =  t.transform.translation.y;
     trans_z1 =  t.transform.translation.z;

     rot_x1 =  t.transform.rotation.x;
     rot_y1 =  t.transform.rotation.y;
     rot_z1 =  t.transform.rotation.z;
     rot_w1 =  t.transform.rotation.w;

    // Build tf2 quaternion from rotation
tf2::Quaternion q1(rot_x1, rot_y1, rot_z1, rot_w1);

// Build tf2 vector from translation
tf2::Vector3 tr1(trans_x1, trans_y1, trans_z1);

// Build a full tf2 transform and inverse
tf1 = tf2::Transform(q1, tr1);
// RCLCPP_INFO(this->get_logger(), "Frame:%s ",frame_child.c_str());

tf1_inv = tf1.inverse();  //(cam relative to tag)
// tf2::Vector3 t_inv = tf_inv.getOrigin(); //get the new vetor postion
// tf2::Quaternion q_inv = tf_inv.getRotation();//get the new roatation

ComputeRealPose();

}}


void GetPose::ComputeRealPose(){
        auto it = tag_world_poses.find(frame_child);
        if (it != tag_world_poses.end()) {
            tf2::Transform Tworld_tag = it->second;
            tf_wc = Tworld_tag * tf1_inv;
            // Tworld_cam = Tworld_tag * Ttag_cam
            tf2::Vector3 trans = tf_wc.getOrigin();
            tf2::Quaternion rot = tf_wc.getRotation();

            double roll, pitch, yaw;
            tf2::Matrix3x3(rot).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                "Position in space (from %s): [x=%.2f, y=%.2f, z=%.2f], RPY(deg) = [Roll: %.2f, Pitch: %.2f, Yaw: %.2f]",
                frame_child.c_str(),
                trans.x(), trans.y(), trans.z(),
                roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI,
                yaw * 180.0 / M_PI
            );
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown tag ID: %s", frame_child.c_str());
        }

    }


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetPose>());
    rclcpp::shutdown();
    return 0;
}

