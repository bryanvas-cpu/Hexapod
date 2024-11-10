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

class HexapodTrajectoryGenNode : public rclcpp::Node {
public:
    HexapodTrajectoryGenNode() : Node("hexapod_trajectory_generator") {
        subscription_joystick = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexapod_controller/array_commands", 10, std::bind(&HexapodTrajectoryGenNode::updateTrajectory, this, _1));
        publisher_ = this->create_publisher<hexapod_interfaces::msg::PointArray>("/trajectory_poses", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / this->publish_frequency)),
                                                std::bind(&HexapodTrajectoryGenNode::publish_leg_positions, this));
    }

private:

    geometry_msgs::msg::Point quadraticBezier3D(geometry_msgs::msg::Point start, geometry_msgs::msg::Point control, geometry_msgs::msg::Point end, double t, int gait) {  // S
        geometry_msgs::msg::Point point;
        t = (t-num_stance_control_points_vec[gait].second)/(this->num_swing_control_points_vec[gait].second);
        double one_minus_t = 1.0-t;
        // Calculate the point on the curve using the formula
        point.x = one_minus_t * one_minus_t * start.x + 2 * one_minus_t * t * control.x + t * t * end.x;
        point.y = one_minus_t * one_minus_t * start.y + 2 * one_minus_t * t * control.y + t * t * end.y;
        point.z = one_minus_t * one_minus_t * start.z + 2 * one_minus_t * t * control.z + t * t * end.z;
        return point;
    }
    geometry_msgs::msg::Point linearBezier3D(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end, double t, int gait) {
        geometry_msgs::msg::Point point;
        t = t/(this->num_stance_control_points_vec[gait].second);
        // Calculate the point on the curve using the linear interpolation formula
        point.x = start.x + t * (end.x - start.x);
        point.y = start.y + t * (end.y - start.y);
        point.z = start.z + t * (end.z - start.z);

        return point;
    }
    void updateTrajectory(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 11) {
            for (size_t i = 0; i < 3; i++) { // lin_vel_x, lin_vel_y, ang_vel_z trans_x, trans_y, trans_z, orient_roll, orient_pitch, orient_yaw, gait, mode
                this->velocity[i] = msg->data[i];
            }
            this->gait = gaits[msg->data[9]];
        } else {
            RCLCPP_WARN(this->get_logger(), "Received target pose message with incorrect size");
        }
    }

    void publish_leg_positions()
    {   
        if(this->t_global > total_control_points)
            this->t_global = 0;

        for(i=0 ; i<6 ; i++){
            update_leg_position(i, this->t_global, this->gait);
        }
        
        hexapod_interfaces::msg::PointArray transformed_leg_positions = transform_leg_positions(leg_positions);

        publisher_->publish(transformed_leg_positions);
        t_global++;
    }

    void update_leg_position(int leg_no, int t_g, int gait){
        generate_3_points(); // sets start, control, end points;
        set_control_point(leg_no, this->offsets[gait].second * leg_no, gait, t_g);
    }

    void generate_3_points() {
        start = { velocity[0], velocity[1], 0.0 }; 
        control = { 0, 0, max_step_height*hypot(velocity[0],velocity[1])/(max_step_length/2)}; 
        end = { -velocity[0],-velocity[1],0.0 };
    }
    
    void set_control_point(int leg_no, double offset, int gait, int t_g){
        geometry_msgs::msg::Point point; 

        if(t_g + offset > this->total_control_points){
            offset = offset - total_control_points;
        }

        if (t_g < this->num_stance_control_points_vec[gait].second) {
            point = linearBezier3D(start, control, t_g + offset, gait);
        } else if (t_g <= this->total_control_points) {
            point = quadraticBezier3D(end, control, start, t_g + offset, gait);
        }
        leg_positions[leg_no] = point;
    }

    hexapod_interfaces::msg::PointArray transform_leg_positions(std::vector<geometry_msgs::msg::Point> leg_positions){
        hexapod_interfaces::msg::PointArray transformed_leg_positions;
        transformed_leg_positions[0].x = leg_positions[0].x += 0.093;
        transformed_leg_positions[0].y = leg_positions[0].y += 0.161;
        transformed_leg_positions[0].z = leg_positions[0].z -= 0.047;

        transformed_leg_positions[1].x = leg_positions[1].x += 0.186;
        transformed_leg_positions[1].y = leg_positions[1].y += 0.0;
        transformed_leg_positions[1].z = leg_positions[1].z -= 0.047;

        transformed_leg_positions[2].x = leg_positions[2].x += 0.093;
        transformed_leg_positions[2].y = leg_positions[2].y -= 0.161;
        transformed_leg_positions[2].z = leg_positions[2].z -= 0.047;

        transformed_leg_positions[3].x = leg_positions[3].x -= 0.093;
        transformed_leg_positions[3].y = leg_positions[3].y -= 0.161;
        transformed_leg_positions[3].z = leg_positions[3].z -= 0.047;

        transformed_leg_positions[4].x = leg_positions[4].x -= 0.186;
        transformed_leg_positions[4].y = leg_positions[4].y += 0.0;
        transformed_leg_positions[4].z = leg_positions[4].z -= 0.047;

        transformed_leg_positions[5].x = leg_positions[5].x -= 0.093;
        transformed_leg_positions[5].y = leg_positions[5].y += 0.161;
        transformed_leg_positions[5].z = leg_positions[5].z -= 0.047;
        return transform_leg_positions;
    }

    std::vector<geometry_msgs::msg::Point> leg_positions(6);

    int t_global=0;

    double velocity[3] = {0,0,0};

    double publish_frequency = 90;
    double total_control_points = 180;


    int gaits[4] = {0,1,2,3}; // 0 = tripod, 1 = wave, 2 = ripple, 3 = amble
    int gait = gaits[0];

    std::vector<std::pair<int,double>> step_swing_ratios {{0,2.0},{1,6.0},{2,3.0},{3,3.0}};
    double step_swing_ratio = step_swing_ratios[gait].second;

    std::vector<std::pair<int,double>> num_stance_control_points_vec {{0,total_control_points/2.0},{1,total_control_points*5.0/6.0},{2,total_control_points*2.0/3.0},{3,total_control_points*2.0/3.0}};
    double num_stance_control_points = num_stance_control_points_vec[gait].second;

    std::vector<std::pair<int,double>> num_swing_control_points_vec {{0,total_control_points/2.0},{1,total_control_points/6.0},{2,total_control_points/3.0},{3,total_control_points/3.0}};
    double num_swing_control_points = num_swing_control_points_vec[gait].second;

    std::vector<std::pair<int,double>> offsets {{0,total_control_points/2.0},{1,total_control_points/6.0},{2,total_control_points/6.0},{3,total_control_points/3.0}};
    double offset = offsets[gait].second;

    geometry_msgs::msg::Point start{0.0, 0.0, 0.0};
    geometry_msgs::msg::Point control{0.0, 0.0, 0.0};
    geometry_msgs::msg::Point end{0.0, 0.0, 0.0};
    double max_step_height = 0.08, max_step_length = 0.1;


    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joystick;
    rclcpp::Publisher<hexapod_interfaces::msg::PointArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodTrajectoryGenNode>());
    rclcpp::shutdown();
    return 0;
}
