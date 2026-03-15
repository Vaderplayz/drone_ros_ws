#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class GetPose : public rclcpp::Node {
public:
GetPose() : Node("PrecLanding") {
    // subscriptions
    subscription_tf1_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        qos,
        std::bind(&GetPose::callbackTagCam, this, std::placeholders::_1)
    );

    subscription_tf2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        qos,
        std::bind(&GetPose::callbackWorldDrone, this, std::placeholders::_1)
    );
    landing_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/landing_target/pose", 10);



    // Define camera wrt drone
    Drone_T_Cam.setOrigin(tf2::Vector3(0.0, 0.0, 0.1));   // 10 cm above drone center
    tf2::Quaternion qtCam;
    qtCam.setRPY(0, M_PI, M_PI/2);  // adjust if camera mounted differently
    Drone_T_Cam.setRotation(qtCam);
    Cam_T_Drone = Drone_T_Cam.inverse();

    // Broadcaster
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // Timer @ 10Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&GetPose::broadcastTFs, this)
    );
}

private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_tf1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_tf2_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr landing_pose_pub_;


    std::string frame_child;
    std::string frame_parent;

    double rollDrone, pitchDrone, yawDrone;


    tf2::Transform Cam_T_Tag;   // tag wrt camera
    tf2::Transform World_T_Tag; // tag wrt world
    tf2::Transform World_T_Drone; // drone wrt world
    tf2::Transform Drone_T_Cam;//cam rela to drone
    tf2::Transform Tag_T_Cam;
    tf2::Transform Cam_T_Drone;

// tag wrt camera
    float trans_x1; 
    float trans_y1;
    float trans_z1;
    float rot_x1;
    float rot_y1;
    float rot_z1;
    float rot_w1;
// tag wrt camera


// drone wrt world
    float trans_x; 
    float trans_y;
    float trans_z;
    float rot_x;
    float rot_y;
    float rot_z;
    float rot_w;
// drone wrt world
 

    double roll;
    double pitch;
    double yaw;

    bool gotDronePose = false;
    bool gotTagPose = false;

    void callbackTagCam(const tf2_msgs::msg::TFMessage::SharedPtr msg); //get tag rela to cam (/tf)
    void callbackWorldDrone(const geometry_msgs::msg::PoseStamped msg); //get drone rela to world
    void tryComputeTagPose();
    void ComputeTagPose(); //only run once when see pose

    void broadcastTFs();

};

void GetPose::callbackWorldDrone(const geometry_msgs::msg::PoseStamped msg){
    trans_x =  msg.pose.position.x;
     trans_y =  msg.pose.position.y;
     trans_z =  msg.pose.position.z;

     rot_x =  msg.pose.orientation.x;
     rot_y =  msg.pose.orientation.y;
     rot_z =  msg.pose.orientation.z;
     rot_w = msg.pose.orientation.w;

tf2::Quaternion qDrone(rot_x, rot_y, rot_z, rot_w);
tf2::Vector3 trDrone(trans_x, trans_y, trans_z);
World_T_Drone = tf2::Transform(qDrone, trDrone);
gotDronePose = true;
tf2::Matrix3x3(qDrone).getRPY(rollDrone, pitchDrone, yawDrone);

tryComputeTagPose();
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->get_clock()->now();
t.header.frame_id = "world";   // parent
t.child_frame_id = "drone";    // child

t.transform.translation.x = msg.pose.position.x;
t.transform.translation.y = msg.pose.position.y;
t.transform.translation.z = msg.pose.position.z;

t.transform.rotation = msg.pose.orientation;

tf_broadcaster_->sendTransform(t);
}



void GetPose::callbackTagCam(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (msg->transforms.empty()) {
        gotTagPose = false;   // no tag detected
        return;
    }

    for (const auto &t : msg->transforms) {
        // Filter for your tag frames
        if (t.child_frame_id.rfind("id", 0) == 0) {  
            trans_x1 = t.transform.translation.x;
            trans_y1 = t.transform.translation.y;
            trans_z1 = t.transform.translation.z;

            rot_x1 = t.transform.rotation.x;
            rot_y1 = t.transform.rotation.y;
            rot_z1 = t.transform.rotation.z;
            rot_w1 = t.transform.rotation.w;

            tf2::Quaternion q1(rot_x1, rot_y1, rot_z1, rot_w1);
            tf2::Vector3 tr1(trans_x1, trans_y1, trans_z1);
            Cam_T_Tag = tf2::Transform(q1, tr1);
            Tag_T_Cam = Cam_T_Tag.inverse();

            // store the frame name
            frame_child = t.child_frame_id;

            gotTagPose = true;
            tryComputeTagPose();
            return; // stop after first tag
        }
    }
    // If no tag frame found in transforms list
    gotTagPose = false;
}

void GetPose::broadcastTFs() {
    rclcpp::Time now = this->get_clock()->now();

    // drone -> camera
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = now;
    t1.header.frame_id = "drone";
    t1.child_frame_id = "camera";
    t1.transform.translation.x = Drone_T_Cam.getOrigin().x();
    t1.transform.translation.y = Drone_T_Cam.getOrigin().y();
    t1.transform.translation.z = Drone_T_Cam.getOrigin().z();
    t1.transform.rotation.x = Drone_T_Cam.getRotation().x();
    t1.transform.rotation.y = Drone_T_Cam.getRotation().y();
    t1.transform.rotation.z = Drone_T_Cam.getRotation().z();
    t1.transform.rotation.w = Drone_T_Cam.getRotation().w();
    tf_broadcaster_->sendTransform(t1);

    // camera -> x500 image (identity, adjust if needed)
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = now;
    t2.header.frame_id = "camera";
    t2.child_frame_id = "x500_mono_cam_down_0/camera_link/imager";
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 0.0;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = 0.0;
    t2.transform.rotation.z = 0.0;
    t2.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t2);
}

 void GetPose::tryComputeTagPose() {
        if (gotDronePose && gotTagPose) {
            ComputeTagPose();
            //gotDronePose = gotTagPose = false;
        }
    }



void GetPose::ComputeTagPose() {
    // Compute World_T_Tag
    World_T_Tag = World_T_Drone * Drone_T_Cam * Cam_T_Tag;

    // Extract translation (position of tag in world frame)
    tf2::Vector3 posTagInWorld = World_T_Tag.getOrigin();

    // Rotation → RPY of tag
    tf2::Quaternion qTagInWorld = World_T_Tag.getRotation();
    double rollTag, pitchTag, yawTag;
    tf2::Matrix3x3(qTagInWorld).getRPY(rollTag, pitchTag, yawTag);

    // // Yaw error
    // double yaw_error = yawTag - yawDrone;
    // // while (yaw_error > M_PI) yaw_error -= 2*M_PI;
    // // while (yaw_error < -M_PI) yaw_error += 2*M_PI;


    //  tf2::Quaternion q_desired;
    // q_desired.setRPY(rollTag, pitchTag, yawTag);  // = yawTag
    // // // q_desired.normalize();

    // Debug info
    RCLCPP_INFO(this->get_logger(),
        "Tag %s in world: [x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f], [x %.3f,y %.3f,z %.3f,w %.3f]",
        frame_child.c_str(),
        posTagInWorld.x(), posTagInWorld.y(), posTagInWorld.z(),
        rollTag, pitchTag, yawTag, qTagInWorld.x(), qTagInWorld.y(), qTagInWorld.z(), qTagInWorld.w());

    // Publish landing_pose
geometry_msgs::msg::PoseStamped landing_pose;
landing_pose.header.stamp = this->now();
landing_pose.header.frame_id = "world"; // ENU

landing_pose.pose.position.x = posTagInWorld.x();  // ENU
landing_pose.pose.position.y = posTagInWorld.y();
landing_pose.pose.position.z = posTagInWorld.z();

landing_pose.pose.orientation.x = qTagInWorld.x();
landing_pose.pose.orientation.y = qTagInWorld.y();
landing_pose.pose.orientation.z = qTagInWorld.z();
landing_pose.pose.orientation.w = qTagInWorld.w();

landing_pose_pub_->publish(landing_pose);
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetPose>());
    rclcpp::shutdown();
    return 0;
}

