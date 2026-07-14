/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan_rf2o");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/lidar/odom_raw");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "lidar_odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", false);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  this->declare_parameter<int>("expected_scan_bins", 720);
  this->get_parameter("expected_scan_bins", expected_scan_bins);
  this->declare_parameter<int>("required_consecutive_valid_scans", 5);
  this->get_parameter("required_consecutive_valid_scans", required_consecutive_valid_scans);
  this->declare_parameter<int>("sustained_invalid_scan_count", 20);
  this->get_parameter("sustained_invalid_scan_count", sustained_invalid_scan_count);
  this->declare_parameter<double>("minimum_finite_return_ratio", 0.05);
  this->get_parameter("minimum_finite_return_ratio", minimum_finite_return_ratio);

  if (expected_scan_bins < 2 || required_consecutive_valid_scans < 1 ||
      sustained_invalid_scan_count < 1 || minimum_finite_return_ratio < 0.0 ||
      minimum_finite_return_ratio > 1.0)
  {
    throw std::invalid_argument("Invalid RF2O canonical scan validation parameters");
  }

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  reset_service = this->create_service<std_srvs::srv::Trigger>(
      "/rf2o/reset", std::bind(&CLaserOdometry2DNode::resetCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
  new_scan_available = false;
}


bool CLaserOdometry2DNode::validateScan(
    const sensor_msgs::msg::LaserScan & scan, std::string & reason)
{
  if (get_publishers_info_by_topic(laser_scan_topic).size() != 1)
  {
    reason = "canonical_scan_publisher_count_not_one";
    return false;
  }
  if (scan.ranges.size() != static_cast<std::size_t>(expected_scan_bins))
  {
    reason = "canonical_scan_width_mismatch";
    return false;
  }
  if (scan.header.frame_id.empty() || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_max) || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment <= 0.0F || !std::isfinite(scan.range_min) ||
      !std::isfinite(scan.range_max) || scan.range_max < scan.range_min)
  {
    reason = "invalid_canonical_scan_metadata";
    return false;
  }

  const rclcpp::Time stamp(scan.header.stamp);
  if (stamp.nanoseconds() <= 0 || (have_input_stamp && stamp <= last_input_stamp))
  {
    reason = "nonmonotonic_canonical_scan_timestamp";
    return false;
  }

  const double declared_span = std::abs(
      static_cast<double>(scan.angle_max) - static_cast<double>(scan.angle_min));
  const double indexed_span = static_cast<double>(scan.angle_increment) *
      static_cast<double>(scan.ranges.size() - 1);
  if (declared_span < 5.8 || indexed_span < 5.8)
  {
    reason = "insufficient_canonical_angular_span";
    return false;
  }

  const std::size_t finite_count = static_cast<std::size_t>(std::count_if(
      scan.ranges.begin(), scan.ranges.end(), [&scan](float range) {
        return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
      }));
  if (static_cast<double>(finite_count) / static_cast<double>(scan.ranges.size()) <
      minimum_finite_return_ratio)
  {
    reason = "insufficient_finite_returns";
    return false;
  }

  if (!have_candidate_geometry)
  {
    candidate_angle_min = scan.angle_min;
    candidate_angle_max = scan.angle_max;
    candidate_angle_increment = scan.angle_increment;
    candidate_frame_id = scan.header.frame_id;
    have_candidate_geometry = true;
  }
  const double tolerance = 1e-5;
  if (scan.header.frame_id != candidate_frame_id ||
      std::abs(scan.angle_min - candidate_angle_min) > tolerance ||
      std::abs(scan.angle_max - candidate_angle_max) > tolerance ||
      std::abs(scan.angle_increment - candidate_angle_increment) > tolerance)
  {
    reason = "unstable_canonical_scan_geometry";
    return false;
  }

  last_input_stamp = stamp;
  have_input_stamp = true;
  reason = "none";
  return true;
}


void CLaserOdometry2DNode::resetEstimator(const std::string & reason)
{
  rf2o_ref.reset();
  new_scan_available = false;
  consecutive_valid_scans = 0;
  consecutive_invalid_scans = 0;
  have_candidate_geometry = false;
  have_input_stamp = false;
  RCLCPP_WARN(get_logger(), "RF2O reset: %s", reason.c_str());
}


void CLaserOdometry2DNode::resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  resetEstimator("explicit_service_request");
  response->success = true;
  response->message = "RF2O reset; waiting for consecutive valid canonical scans";
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    std::string rejection_reason;
    if (!validateScan(*new_scan, rejection_reason))
    {
      consecutive_valid_scans = 0;
      ++consecutive_invalid_scans;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Skipping invalid canonical scan: %s",
        rejection_reason.c_str());
      if (rf2o_ref.is_initialized() &&
          consecutive_invalid_scans >= static_cast<std::size_t>(sustained_invalid_scan_count))
      {
        resetEstimator("sustained_invalid_canonical_input");
      }
      return;
    }

    consecutive_invalid_scans = 0;
    ++consecutive_valid_scans;
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;

    if (rf2o_ref.first_laser_scan == false)
    {
      for (unsigned int i = 0; i < rf2o_ref.width; i++)
        rf2o_ref.range_wf(i) = new_scan->ranges[i];
      // inform of new scan available
      new_scan_available = true;
    }
    else
    {
      if (consecutive_valid_scans < static_cast<std::size_t>(required_consecutive_valid_scans))
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "RF2O canonical scan qualification: %zu/%d",
          consecutive_valid_scans, required_consecutive_valid_scans);
        return;
      }
      // Initialize module on first scan (from laser params)
      if (!setLaserPoseFromTf())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for transform from %s to %s before RF2O initialization",
          last_scan.header.frame_id.c_str(), base_frame_id.c_str());
        return;
      }
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      rf2o_ref.first_laser_scan = false;
    }
  }
}


/**
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    retrieved = false;
  }

  if (!retrieved)
    return false;

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    // Process odometry estimation
    if (rf2o_ref.odometryCalculation(last_scan))
    {
      // Publish only when RF2O accepted and processed this scan.
      publish();
    }

    // Do not run on the same data!
    new_scan_available = false;
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for qualified laser scans");
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);

  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  //publish the message
  odom_pub->publish(odom);

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok()){
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }

  return 0;
}
