#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class ComparePoseRPY : public rclcpp::Node
{
public:
    ComparePoseRPY() : Node("compare_pose_rpy")
    {
        sub_vision_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10,
            std::bind(&ComparePoseRPY::visionCallback, this, std::placeholders::_1));

        sub_local_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", 10,
            std::bind(&ComparePoseRPY::localCallback, this, std::placeholders::_1));
    }

private:
    void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        printRPY("VISION", msg->pose.orientation);
    }

    void localCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        printRPY("LOCAL ", msg->pose.orientation);
    }

    void printRPY(const std::string &label, const geometry_msgs::msg::Quaternion &q_msg)
    {
        tf2::Quaternion q(
            q_msg.x,
            q_msg.y,
            q_msg.z,
            q_msg.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
                    "[%s] RPY = (%.3f, %.3f, %.3f)",
                    label.c_str(), roll, pitch, yaw);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vision_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComparePoseRPY>());
    rclcpp::shutdown();
    return 0;
}
