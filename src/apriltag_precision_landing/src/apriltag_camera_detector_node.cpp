#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromString(const std::string &name) {
  if (name == "16h5") return cv::aruco::DICT_APRILTAG_16h5;
  if (name == "25h9") return cv::aruco::DICT_APRILTAG_25h9;
  if (name == "36h10") return cv::aruco::DICT_APRILTAG_36h10;
  return cv::aruco::DICT_APRILTAG_36h11;
}

double quadArea(const std::vector<cv::Point2f> &c) {
  if (c.size() != 4) return 0.0;
  return std::fabs(cv::contourArea(c));
}

double pixelAreaToSquareMeters(double area_px, double z_m, double fx, double fy) {
  if (area_px <= 0.0 || z_m <= 0.0 || fx <= 1e-9 || fy <= 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return area_px * (z_m * z_m) / (fx * fy);
}
}  // namespace

class AprilTagCameraDetectorNode : public rclcpp::Node {
 public:
  AprilTagCameraDetectorNode() : Node("apriltag_camera_detector") {
    input_source_ = declare_parameter<std::string>("input_source", "device");

    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera_link");

    video_device_ = declare_parameter<std::string>("video_device", "/dev/video1");
    device_width_ = declare_parameter<int>("device_width", 640);
    device_height_ = declare_parameter<int>("device_height", 480);
    device_fps_ = declare_parameter<double>("device_fps", 30.0);
    detect_rate_hz_ = declare_parameter<double>("detect_rate_hz", 20.0);

    fx_ = declare_parameter<double>("fx", 0.0);
    fy_ = declare_parameter<double>("fy", 0.0);
    cx_ = declare_parameter<double>("cx", 0.0);
    cy_ = declare_parameter<double>("cy", 0.0);
    hfov_deg_ = declare_parameter<double>("hfov_deg", 78.0);
    dist_coeffs_vec_ = declare_parameter<std::vector<double>>("dist_coeffs", std::vector<double>{});

    tag_pose_topic_ = declare_parameter<std::string>("tag_pose_topic", "/precision_landing/tag_pose_camera");
    tag_size_m_ = declare_parameter<double>("tag_size_m", 0.12);
    target_tag_id_ = declare_parameter<int>("target_tag_id", -1);
    min_tag_area_px_ = declare_parameter<double>("min_tag_area_px", 80.0);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "36h11");

    detector_dict_ = cv::aruco::getPredefinedDictionary(dictionaryFromString(dictionary_name_));
    pub_tag_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(tag_pose_topic_, 10);

    if (tag_size_m_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "tag_size_m must be > 0.0");
      throw std::runtime_error("invalid tag_size_m");
    }

    if (input_source_ == "ros_topics") {
      initRosTopicMode();
    } else if (input_source_ == "device") {
      initDeviceMode();
    } else {
      RCLCPP_FATAL(get_logger(), "Unknown input_source=%s (use ros_topics or device)", input_source_.c_str());
      throw std::runtime_error("invalid input_source");
    }

    RCLCPP_INFO(get_logger(),
                "apriltag_camera_detector started source=%s dict=%s tag_size=%.3f out=%s",
                input_source_.c_str(), dictionary_name_.c_str(), tag_size_m_, tag_pose_topic_.c_str());
  }

  ~AprilTagCameraDetectorNode() override {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

 private:
  void initRosTopicMode() {
    const auto qos_sensor = rclcpp::SensorDataQoS();
    sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::cameraInfoCb, this, std::placeholders::_1));

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::imageCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ROS-topic mode image=%s camera_info=%s",
                image_topic_.c_str(), camera_info_topic_.c_str());
  }

  void initDeviceMode() {
    setupCameraModelFromParams();

    cap_.open(video_device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      cap_.open(video_device_);
    }

    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Cannot open video device: %s", video_device_.c_str());
      throw std::runtime_error("video device open failed");
    }

    if (device_width_ > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(device_width_));
    if (device_height_ > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(device_height_));
    if (device_fps_ > 0.0) cap_.set(cv::CAP_PROP_FPS, device_fps_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, detect_rate_hz_));
    capture_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AprilTagCameraDetectorNode::captureLoop, this));

    RCLCPP_INFO(get_logger(), "Device mode video=%s (requested %dx%d @ %.1f)",
                video_device_.c_str(), device_width_, device_height_, device_fps_);
  }

  void setupCameraModelFromParams() {
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;

    if (fx <= 0.0 || fy <= 0.0) {
      const double width = device_width_ > 0 ? static_cast<double>(device_width_) : 640.0;
      const double hfov_rad = std::max(1.0, hfov_deg_) * M_PI / 180.0;
      const double fx_auto = width / (2.0 * std::tan(hfov_rad * 0.5));
      fx = fy = fx_auto;
      RCLCPP_WARN(get_logger(),
                  "fx/fy not provided. Using approximate intrinsics from hfov_deg=%.1f -> fx=fy=%.1f",
                  hfov_deg_, fx_auto);
    }

    if (cx <= 0.0 || cy <= 0.0) {
      const double width = device_width_ > 0 ? static_cast<double>(device_width_) : 640.0;
      const double height = device_height_ > 0 ? static_cast<double>(device_height_) : 480.0;
      cx = width * 0.5;
      cy = height * 0.5;
      RCLCPP_WARN(get_logger(),
                  "cx/cy not provided. Using image center approximation cx=%.1f cy=%.1f", cx, cy);
    }

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = fx;
    camera_matrix_.at<double>(0, 2) = cx;
    camera_matrix_.at<double>(1, 1) = fy;
    camera_matrix_.at<double>(1, 2) = cy;
    camera_matrix_.at<double>(2, 2) = 1.0;

    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(dist_coeffs_vec_.size()), CV_64F);
    for (size_t i = 0; i < dist_coeffs_vec_.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_coeffs_vec_[i];
    }

    got_camera_info_ = true;
  }

  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.size() != 9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "CameraInfo K is invalid (size=%zu)", msg->k.size());
      return;
    }

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];
    camera_matrix_.at<double>(0, 1) = msg->k[1];
    camera_matrix_.at<double>(0, 2) = msg->k[2];
    camera_matrix_.at<double>(1, 0) = msg->k[3];
    camera_matrix_.at<double>(1, 1) = msg->k[4];
    camera_matrix_.at<double>(1, 2) = msg->k[5];
    camera_matrix_.at<double>(2, 0) = msg->k[6];
    camera_matrix_.at<double>(2, 1) = msg->k[7];
    camera_matrix_.at<double>(2, 2) = msg->k[8];

    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }

    if (!msg->header.frame_id.empty()) {
      camera_frame_id_ = msg->header.frame_id;
    }

    got_camera_info_ = true;
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!got_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting camera_info before AprilTag detection.");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cv_bridge exception: %s", e.what());
      return;
    }

    detectAndPublish(cv_ptr->image, rclcpp::Time(msg->header.stamp), msg->header.frame_id);
  }

  void captureLoop() {
    if (!cap_.isOpened()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Video device not open: %s", video_device_.c_str());
      return;
    }

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to read frame from %s", video_device_.c_str());
      return;
    }

    detectAndPublish(frame, now(), camera_frame_id_);
  }

  void detectAndPublish(const cv::Mat &image, const rclcpp::Time &stamp, const std::string &frame_id) {
    cv::Mat gray;
    if (image.channels() == 1) {
      gray = image;
    } else {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(gray, detector_dict_, corners, ids, detector_params, rejected);

    if (ids.empty()) {
      return;
    }

    int best_idx = -1;
    double best_area_px = 0.0;

    for (size_t i = 0; i < ids.size(); ++i) {
      if (target_tag_id_ >= 0 && ids[i] != target_tag_id_) {
        continue;
      }

      const double area_px = quadArea(corners[i]);
      if (area_px < min_tag_area_px_) {
        continue;
      }

      if (best_idx < 0 || area_px > best_area_px) {
        best_idx = static_cast<int>(i);
        best_area_px = area_px;
      }
    }

    if (best_idx < 0) {
      return;
    }

    std::vector<std::vector<cv::Point2f>> picked_corners{corners[best_idx]};
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        picked_corners,
        static_cast<float>(tag_size_m_),
        camera_matrix_,
        dist_coeffs_,
        rvecs,
        tvecs);

    if (rvecs.empty() || tvecs.empty()) {
      return;
    }

    const cv::Vec3d rvec = rvecs[0];
    const cv::Vec3d tvec = tvecs[0];

    const double angle = std::sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (angle > 1e-9) {
      const double ax = rvec[0] / angle;
      const double ay = rvec[1] / angle;
      const double az = rvec[2] / angle;
      const double s = std::sin(angle * 0.5);
      qx = ax * s;
      qy = ay * s;
      qz = az * s;
      qw = std::cos(angle * 0.5);
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id.empty() ? camera_frame_id_ : frame_id;

    out.pose.position.x = tvec[0];
    out.pose.position.y = tvec[1];
    out.pose.position.z = tvec[2];
    out.pose.orientation.x = qx;
    out.pose.orientation.y = qy;
    out.pose.orientation.z = qz;
    out.pose.orientation.w = qw;

    pub_tag_pose_->publish(out);

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double z_abs = std::fabs(out.pose.position.z);
    const double area_m2 = pixelAreaToSquareMeters(best_area_px, z_abs, fx, fy);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Tag detected id=%d pos_cam=[%.2f %.2f %.2f] area_m2=%.6f",
                         ids[best_idx], out.pose.position.x, out.pose.position.y,
                         out.pose.position.z, area_m2);
  }

  std::string input_source_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_id_;

  std::string video_device_;
  int device_width_{640};
  int device_height_{480};
  double device_fps_{30.0};
  double detect_rate_hz_{20.0};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  double hfov_deg_{78.0};
  std::vector<double> dist_coeffs_vec_;

  std::string tag_pose_topic_;
  std::string dictionary_name_;

  double tag_size_m_{0.16};
  int target_tag_id_{-1};
  double min_tag_area_px_{80.0};

  bool got_camera_info_{false};

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> detector_dict_;
  cv::VideoCapture cap_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tag_pose_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagCameraDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
