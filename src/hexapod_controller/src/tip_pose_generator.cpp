#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/Imu.hpp"
#include <array>
#include <vector>
#include <utility>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "pid_controller.hpp"
#include "hexapod_interfaces/msg/point_array.hpp"

using namespace std::placeholders;

class HexapodTipPoseGenNode : public rclcpp::Node {
public:
    HexapodTipPoseGenNode() : Node("hexapod_tip_pose_generator") {
        subscription_tip_trajectory = this->create_subscription<hexapod_interfaces::msg::PointArray>(
            "/trajectory_poses", 10, std::bind(&HexapodTipPoseGenNode::update_trajectory_points, this, _1));
        subscription_body_pose = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pid_pose_commands", 10, std::bind(&HexapodTipPoseGenNode::update_pose_points, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tip_poses", 10);
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_tip_trajectory;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_body_pose;
    rclcpp::Publisher<hexapod_interfaces::msg::PointArray>::SharedPtr publisher_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodTipPoseGenNode>());
    rclcpp::shutdown();
    return 0;
}
