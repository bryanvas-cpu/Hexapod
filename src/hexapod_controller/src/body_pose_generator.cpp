#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include <array>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "hexapod_controller/pid_controller.hpp"

using namespace std::placeholders;

class HexapodManualPoseNode : public rclcpp::Node {
public:
    HexapodManualPoseNode() : Node("hexapod_manual_pose_generator"),
    roll(Kp[0], Ki[0], Kd[0]),
    pitch(Kp[1], Ki[1], Kd[1]) {
        subscription_joystick = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexapod_controller/joy_commands", 10, std::bind(&HexapodManualPoseNode::updateTargetPoseValues, this, _1));
        subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/out", 10, std::bind(&HexapodManualPoseNode::updateCurrentPoseValues, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pid_pose_commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / this->publish_frequency)),
                                                std::bind(&HexapodManualPoseNode::publish_rpy_xyz, this));
    }

private:
    void updateTargetPoseValues(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 11) {
            for (size_t i = 0; i < 3; ++i) { // lin_vel_x, lin_vel_y, ang_vel_z trans_x, trans_y, trans_z, orient_roll, orient_pitch, orient_yaw, gait, mode
                this->target_orient_values[i] = msg->data[i+6];
                this->target_transl_values[i] = msg->data[i+3];
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received target pose message with incorrect size");
        }

        // RCLCPP_INFO(this->get_logger(),"rpy xyz _targets  (%.2f,%.2f,%.2f),  (%.2f,%.2f,%.2f)", target_orient_values[0], target_orient_values[1], target_orient_values[2], target_transl_values[0], target_transl_values[1], target_transl_values[2]);

    }

    void updateCurrentPoseValues(const sensor_msgs::msg::Imu& msg)
    {
        // Assuming that current_pose_values represents roll, pitch, yaw, and x, y, z
        double roll, pitch, yaw;

        // Convert quaternion to roll, pitch, and yaw
        tf2::Quaternion q(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        this->current_orient_values[0] = roll;
        this->current_orient_values[1] = pitch;
        this->current_orient_values[2] = yaw;
        // RCLCPP_INFO(this->get_logger(),"rpy according to IMU (%.2f,%.2f,%.2f)", current_orient_values[0], current_orient_values[1], current_orient_values[2]);
    }

    void publish_rpy_xyz()
    {
        std_msgs::msg::Float64MultiArray msg;

        double control_signal_roll = this->roll.compute(target_orient_values[0], current_orient_values[0], 1.0 /this->publish_frequency);
        double control_signal_pitch = this->pitch.compute(target_orient_values[1], current_orient_values[1], 1.0/this->publish_frequency);

        // INCASE PID HAS TO BE USED
        msg.data = {this-> current_orient_values[0] + (1/publish_frequency)*control_signal_roll, 
                    this-> current_orient_values[1] + (1/publish_frequency)*control_signal_pitch, 
                    this->target_orient_values[2],
                    this->target_transl_values[0],
                    this->target_transl_values[1],
                    this->target_transl_values[2]
                    };

        // msg.data = {target_orient_values[0] - current_orient_values[0], 
        //             target_orient_values[1] - current_orient_values[1], 
        //             target_orient_values[2],
        //             this->target_transl_values[0],
        //             this->target_transl_values[1],
        //             this->target_transl_values[2]
        //             };
        // msg.data = {target_orient_values[0] , 
        //             target_orient_values[1] , 
        //             target_orient_values[2],
        //             this->target_transl_values[0],
        //             this->target_transl_values[1],
        //             this->target_transl_values[2]
        //             };

        // RCLCPP_INFO(this->get_logger(),"rpy_commands_target: %.4f, %.4f", target_orient_values[0], target_orient_values[1]);

        // RCLCPP_INFO(this->get_logger(),"rpy_commands_current:            %.4f, %.4f", current_orient_values[0], current_orient_values[1]);


        // RCLCPP_INFO(this->get_logger(),"rpy_commands_sent\n roll: %.4f,\npitch: %.4f,\nyaw: %.4f", target_orient_values[0] - current_orient_values[0], target_orient_values[1] - current_orient_values[1], target_orient_values[2]);
   //     // RCLCPP_INFO(this->get_logger(),"xyz_commands_sent\n roll: %.4f,\npitch: %.4f,\nyaw: %.4f", target_transl_values[0] - current_orient_values[0], target_transl_values[1] - current_orient_values[1], target_transl_values[2]);


        publisher_->publish(msg);
    }

    double Kp[2] = {125, 125}; // r p y
    double Ki[2] = {0.1, 0.1};
    double Kd[2] = {1.0, 1.0};

    PID roll;
    PID pitch;
    
    std::vector<double> target_orient_values = {0,0,0};
    std::vector<double> target_transl_values = {0,0,0};
    std::vector<double> current_orient_values = {0,0,0};
    double publish_frequency = 100;

    float coxa_length = 0, femur_length = 0.08, tibia_length = 0.177;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joystick;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodManualPoseNode>());
    rclcpp::shutdown();
    return 0;
}
