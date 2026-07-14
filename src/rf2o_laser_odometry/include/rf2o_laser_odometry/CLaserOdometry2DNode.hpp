#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <std_srvs/srv/trigger.hpp>

namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  void process();
  void publish();
  bool setLaserPoseFromTf();
  bool scan_available();
  bool validateScan(const sensor_msgs::msg::LaserScan & scan, std::string & reason);
  void resetEstimator(const std::string & reason);

  // Params & vars
  CLaserOdometry2D    rf2o_ref;
  bool                publish_tf, new_scan_available;
  double              freq;
  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;
  int                 expected_scan_bins, required_consecutive_valid_scans;
  int                 sustained_invalid_scan_count;
  double              minimum_finite_return_ratio;

  sensor_msgs::msg::LaserScan                     last_scan;
  bool                                            GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer>                buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>  odom_broadcaster;
  nav_msgs::msg::Odometry                         initial_robot_pose;

  // Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             reset_service;

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Time last_input_stamp{0, 0, RCL_ROS_TIME};
  bool have_input_stamp{false};
  bool have_candidate_geometry{false};
  std::size_t consecutive_valid_scans{0};
  std::size_t consecutive_invalid_scans{0};
  float candidate_angle_min{0.0F};
  float candidate_angle_max{0.0F};
  float candidate_angle_increment{0.0F};
  std::string candidate_frame_id;
};
