#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace {
inline bool starts_with(const std::string &value, const std::string &prefix) {
  return value.rfind(prefix, 0) == 0;
}
}  // namespace

class AprilTagPrecisionLandingNode : public rclcpp::Node {
 public:
  AprilTagPrecisionLandingNode() : Node("apriltag_precision_landing") {
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    camera_tag_pose_topic_ = declare_parameter<std::string>("camera_tag_pose_topic", "/precision_landing/tag_pose_camera");
    relay_image_stream_ = declare_parameter<bool>("relay_image_stream", true);
    image_input_topic_ = declare_parameter<std::string>("image_input_topic", "/camera/image_raw");
    image_output_topic_ = declare_parameter<std::string>("image_output_topic", "/image_raw");
    drone_pose_topic_ = declare_parameter<std::string>("drone_pose_topic", "/mavros/local_position/pose");
    landing_target_topic_ = declare_parameter<std::string>("landing_target_topic", "/mavros/landing_target/pose");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");

    input_mode_ = declare_parameter<std::string>("input_mode", "camera_pose");
    tag_frame_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "id");
    tag_frame_exact_ = declare_parameter<std::string>("tag_frame_exact", "");

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.30);

    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.0);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.0);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.0);
    camera_roll_ = declare_parameter<double>("camera_roll", 0.0);
    camera_pitch_ = declare_parameter<double>("camera_pitch", M_PI);
    camera_yaw_ = declare_parameter<double>("camera_yaw", M_PI_2);

    publish_debug_tf_ = declare_parameter<bool>("publish_debug_tf", true);
    debug_tag_child_frame_ = declare_parameter<std::string>("debug_tag_child_frame", "apriltag_detected");

    tf2::Quaternion q_cam;
    q_cam.setRPY(camera_roll_, camera_pitch_, camera_yaw_);
    drone_t_cam_.setOrigin(tf2::Vector3(camera_offset_x_, camera_offset_y_, camera_offset_z_));
    drone_t_cam_.setRotation(q_cam);

    const auto qos_sensor = rclcpp::SensorDataQoS();

    if (relay_image_stream_) {
      if (image_input_topic_ == image_output_topic_) {
        RCLCPP_INFO(get_logger(),
                    "image relay enabled but input==output (%s), skipping relay to avoid topic loop",
                    image_output_topic_.c_str());
      } else {
        pub_image_relay_ = create_publisher<sensor_msgs::msg::Image>(image_output_topic_, qos_sensor);
        sub_image_relay_ = create_subscription<sensor_msgs::msg::Image>(
            image_input_topic_, qos_sensor,
            std::bind(&AprilTagPrecisionLandingNode::imageRelayCb, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Relaying image stream %s -> %s",
                    image_input_topic_.c_str(), image_output_topic_.c_str());
      }
    }

    if (input_mode_ == "tf" || input_mode_ == "auto") {
      sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
          tf_topic_, qos_sensor,
          std::bind(&AprilTagPrecisionLandingNode::tfCallback, this, std::placeholders::_1));
    }

    if (input_mode_ == "camera_pose" || input_mode_ == "auto") {
      sub_camera_tag_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          camera_tag_pose_topic_, qos_sensor,
          std::bind(&AprilTagPrecisionLandingNode::cameraTagPoseCb, this, std::placeholders::_1));
    }

    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        drone_pose_topic_, qos_sensor,
        std::bind(&AprilTagPrecisionLandingNode::dronePoseCallback, this, std::placeholders::_1));

    pub_landing_target_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(landing_target_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AprilTagPrecisionLandingNode::publishLandingPose, this));

    RCLCPP_INFO(get_logger(),
                "apriltag_precision_landing started mode=%s tf_topic=%s camera_tag_pose_topic=%s",
                input_mode_.c_str(), tf_topic_.c_str(), camera_tag_pose_topic_.c_str());
  }

 private:
  bool frameMatchesTag(const std::string &frame_id) const {
    if (!tag_frame_exact_.empty()) {
      return frame_id == tag_frame_exact_;
    }
    if (tag_frame_prefix_.empty()) {
      return true;
    }
    return starts_with(frame_id, tag_frame_prefix_);
  }

  void dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Vector3 t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    world_t_drone_ = tf2::Transform(q, t);

    if (!msg->header.frame_id.empty()) {
      world_frame_from_pose_ = msg->header.frame_id;
    }

    got_world_t_drone_ = true;
    last_drone_pose_time_ = now();
  }

  void imageRelayCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!pub_image_relay_) {
      return;
    }
    auto out = *msg;
    if (out.header.frame_id.empty() && !camera_frame_id_.empty()) {
      out.header.frame_id = camera_frame_id_;
    }
    pub_image_relay_->publish(out);
  }

  void cameraTagPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Vector3 t(msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z);

    cam_t_tag_ = tf2::Transform(q, t);
    got_cam_t_tag_ = true;
    last_tag_time_ = now();
    input_source_ = "camera_pose";
    camera_frame_id_ = msg->header.frame_id;
    tag_frame_id_ = tag_frame_exact_.empty() ? "apriltag" : tag_frame_exact_;
  }

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (msg->transforms.empty()) {
      return;
    }

    for (const auto &tr : msg->transforms) {
      if (!frameMatchesTag(tr.child_frame_id)) {
        continue;
      }

      tf2::Quaternion q(tr.transform.rotation.x,
                        tr.transform.rotation.y,
                        tr.transform.rotation.z,
                        tr.transform.rotation.w);
      tf2::Vector3 t(tr.transform.translation.x,
                     tr.transform.translation.y,
                     tr.transform.translation.z);

      cam_t_tag_ = tf2::Transform(q, t);
      got_cam_t_tag_ = true;
      last_tag_time_ = now();
      input_source_ = "tf";
      tag_frame_id_ = tr.child_frame_id;
      camera_frame_id_ = tr.header.frame_id;
      return;
    }
  }

  bool stale(const rclcpp::Time &stamp) const {
    return (now() - stamp).seconds() > input_timeout_sec_;
  }

  void publishLandingPose() {
    if (!got_world_t_drone_ || !got_cam_t_tag_) {
      return;
    }

    if (stale(last_drone_pose_time_) || stale(last_tag_time_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Tag/drone pose stale (timeout %.2fs).", input_timeout_sec_);
      return;
    }

    const tf2::Transform world_t_tag = world_t_drone_ * drone_t_cam_ * cam_t_tag_;

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = now();
    out.header.frame_id = world_frame_from_pose_.empty() ? world_frame_ : world_frame_from_pose_;
    out.pose.position.x = world_t_tag.getOrigin().x();
    out.pose.position.y = world_t_tag.getOrigin().y();
    out.pose.position.z = world_t_tag.getOrigin().z();
    out.pose.orientation.x = world_t_tag.getRotation().x();
    out.pose.orientation.y = world_t_tag.getRotation().y();
    out.pose.orientation.z = world_t_tag.getRotation().z();
    out.pose.orientation.w = world_t_tag.getRotation().w();

    pub_landing_target_pose_->publish(out);

    if (publish_debug_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = out.header;
      tf_msg.child_frame_id = debug_tag_child_frame_;
      tf_msg.transform.translation.x = out.pose.position.x;
      tf_msg.transform.translation.y = out.pose.position.y;
      tf_msg.transform.translation.z = out.pose.position.z;
      tf_msg.transform.rotation = out.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Target source=%s frame=%s pos=[%.2f %.2f %.2f]",
                         input_source_.c_str(), camera_frame_id_.c_str(),
                         out.pose.position.x, out.pose.position.y, out.pose.position.z);
  }

  std::string tf_topic_;
  std::string camera_tag_pose_topic_;
  bool relay_image_stream_{true};
  std::string image_input_topic_;
  std::string image_output_topic_;
  std::string drone_pose_topic_;
  std::string landing_target_topic_;
  std::string world_frame_;
  std::string input_mode_;
  std::string input_source_{"none"};
  std::string tag_frame_prefix_;
  std::string tag_frame_exact_;
  std::string debug_tag_child_frame_;

  double input_timeout_sec_{0.30};
  double camera_offset_x_{0.0};
  double camera_offset_y_{0.0};
  double camera_offset_z_{0.0};
  double camera_roll_{0.0};
  double camera_pitch_{M_PI};
  double camera_yaw_{M_PI_2};

  bool publish_debug_tf_{true};
  bool got_world_t_drone_{false};
  bool got_cam_t_tag_{false};

  std::string world_frame_from_pose_;
  std::string tag_frame_id_;
  std::string camera_frame_id_;

  tf2::Transform world_t_drone_;
  tf2::Transform cam_t_tag_;
  tf2::Transform drone_t_cam_;

  rclcpp::Time last_drone_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tag_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_relay_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_camera_tag_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_relay_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_landing_target_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagPrecisionLandingNode>());
  rclcpp::shutdown();
  return 0;
}
