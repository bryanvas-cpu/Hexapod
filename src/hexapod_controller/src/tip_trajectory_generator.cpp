#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <unistd.h>
#include "geometry_msgs/msg/point_stamped.hpp"
// #include "sensor_msgs/msg/Imu.hpp"
// #include <array>
#include <vector>
// #include <utility>
#include <cmath>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include "pid_controller.hpp"
#include "hexapod_interfaces/msg/point_array.hpp"

using namespace std::placeholders;

class HexapodTrajectoryGenNode : public rclcpp::Node {
public:
    HexapodTrajectoryGenNode() : Node("hexapod_trajectory_generator") {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Reached constructor _1");

        subscription_joystick = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexapod_controller/joy_commands", 10, std::bind(&HexapodTrajectoryGenNode::updateTrajectory, this, _1));
        
        // RCLCPP_INFO_STREAM(this->get_logger(), "created subscription_joystick subscriber");

        publisher_ = this->create_publisher<hexapod_interfaces::msg::PointArray>("/trajectory_poses", 10);

        // RCLCPP_INFO_STREAM(this->get_logger(), "created publisher_ publisher");

        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / this->publish_frequency)),
                                                std::bind(&HexapodTrajectoryGenNode::publish_leg_positions, this));
        
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped0", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped1", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped2", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped3", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped4", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/point_stamped5", 10));

        // RCLCPP_INFO_STREAM(this->get_logger(), "created publisher_ publisher");


        // RCLCPP_INFO_STREAM(this->get_logger(), "created created timer");

    }

private:

    geometry_msgs::msg::Point quadraticBezier3D(geometry_msgs::msg::Point start, geometry_msgs::msg::Point control, geometry_msgs::msg::Point end, double t, int gait) {  // S
        geometry_msgs::msg::Point point;
        t = (t-num_stance_control_points_vec[gait].second)/(this->num_swing_control_points_vec[gait].second);
        // RCLCPP_INFO(this->get_logger(), "t_quadratic_bezier_3d %f", t);

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
        // RCLCPP_INFO(this->get_logger(), "t_linear_bezier_3d %f",t);

        // Calculate the point on the curve using the linear interpolation formula
        point.x = start.x + t * (end.x - start.x);
        point.y = start.y + t * (end.y - start.y);
        point.z = start.z + t * (end.z - start.z);

        return point;
    }
    void updateTrajectory(const std_msgs::msg::Float64MultiArray& msg)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "update trraectory calback called");

        if (msg.data.size() == 11) {
            for (int i = 0; i < 3; i++) { // lin_vel_x, lin_vel_y, ang_vel_z trans_x, trans_y, trans_z, orient_roll, orient_pitch, orient_yaw, gait, mode
                this->velocity[i] = msg.data[i];
                // RCLCPP_INFO(this->get_logger(),"velocity received %f", this->velocity[i]);
            }
            this->gait = (int)(msg.data[9]);
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Received target pose message with incorrect size");
        }

        // RCLCPP_INFO(this->get_logger(), "lin_xy, ang_z velocity  %f, %f, - %f", this->velocity[0], velocity[1], velocity[2]);

    }

    void publish_leg_positions()
    {   
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered publish leg positions");

        if(this->t_global > this->total_control_points)
            this->t_global = 0;

        for(int i=0 ; i<6 ; i++){
            update_leg_position(i, this->t_global, this->gait);
        }
        
        hexapod_interfaces::msg::PointArray transformed_leg_positions = transform_leg_positions(leg_positions);

        for(int i=0 ; i<6 ; i++){
            // RCLCPP_INFO(this->get_logger(), "leg[%d] position (%.3f,%.3f,%.3f)",i,transformed_leg_positions.points[i].x,transformed_leg_positions.points[i].y,transformed_leg_positions.points[i].z);
        }


        for(int i=0; i<6 ; i++){
            geometry_msgs::msg::PointStamped leg_point;

            leg_point.header.frame_id = "body";  // Set the frame of reference
            leg_point.header.stamp = this->get_clock()->now();

            leg_point.point.x = transformed_leg_positions.points[i].y;
            leg_point.point.y = -transformed_leg_positions.points[i].x;
            leg_point.point.z = transformed_leg_positions.points[i].z;
            publisher_point_stamped[i]->publish(leg_point);
        }
        publisher_->publish(transformed_leg_positions);
        // RCLCPP_INFO(this->get_logger(), "t_global  %d", this->t_global);

        this->t_global++;

        // RCLCPP_INFO_STREAM(this->get_logger(), "finished publish leg positions");

    }

    void update_leg_position(int leg_no, int t_g, int gait){

        // RCLCPP_INFO_STREAM(this->get_logger(), "entered update leg position");

        generate_3_points(); // sets start, control, end points;

        // RCLCPP_INFO(this->get_logger(), "start: (%f, %f, %f) \n control: (%f, %f, %f) \n end: (%f, %f, %f)", this->start.x, this->start.y, this->start.z, this->control.x, this->control.y, this->control.z, this->end.x, this->end.y, this->end.z);

        set_control_point(leg_no, this->offsets[gait].second * leg_no, gait, t_g);
        
        

        // RCLCPP_INFO_STREAM(this->get_logger(), "finished update leg position");
    }

    void generate_3_points() {

        // RCLCPP_INFO_STREAM(this->get_logger(), "entered generate 3 points");

        this->start.x = (0.5 * velocity[0] * this->num_stance_control_points_vec[gait].second / publish_frequency) * (num_stance_control_points_vec[0].second/num_stance_control_points_vec[gait].second); // (num_stance_control_points[0].second/num_stance_control_points[gait].second) is correction for step size when gait is not tripod. its inverse is the correction factor for vwlocity when not in tripod gate
        this->start.y = (0.5 * velocity[1] * this->num_stance_control_points_vec[gait].second / publish_frequency) * (num_stance_control_points_vec[0].second/num_stance_control_points_vec[gait].second);
        this->start.z = 0.0;

        this->end.x = -this->start.x;
        this->end.y = -this->start.y;
        this->end.z = 0.0;

        this->control.x = 0.0;
        this->control.y = 0.0;
        this->control.z = 1.0 * hypot(start.x - end.x, start.y - end.y);

  //      // RCLCPP_INFO(this->get_logger(),"start, control, end (%f, %f, %f) (%f, %f, %f) (%f, %f, %f)", this->start.x, this->start.y, this->start.z, this->control.x, this->control.y, this->control.z, this->end.x, this->end.y, this->end.z);
        // RCLCPP_INFO_STREAM(this->get_logger(), "finished generate 3 points");
}
    
    void set_control_point(int leg_no, double offset, int gait, int t_g){

        // RCLCPP_INFO_STREAM(this->get_logger(), "entered set control point");

        geometry_msgs::msg::Point point; 

        while(offset >= this->total_control_points){
            offset = offset - total_control_points;
        }

        while(t_g + offset >total_control_points){
            t_g -= total_control_points;
        }

        // RCLCPP_INFO(this->get_logger(), "offset: %f", offset);

        if (t_g + offset < this->num_stance_control_points_vec[gait].second) {
            point = linearBezier3D(start, end, t_g + offset, gait);
        } else if (t_g + offset <= this->total_control_points) {
            point = quadraticBezier3D(end, control, start, t_g + offset, gait);
        }
        
        leg_positions[leg_no] = point;
        // RCLCPP_INFO(this->get_logger(), "leg[%d] position (%.3f,%.3f,%.3f)",leg_no,point.x,point.y,point.z);
        // usleep(100000);
        // RCLCPP_INFO_STREAM(this->get_logger(), "finished set control point");

    }

    hexapod_interfaces::msg::PointArray transform_leg_positions(std::vector<geometry_msgs::msg::Point>& leg_positions){

        // RCLCPP_INFO_STREAM(this->get_logger(), "entered transform_leg_position");

        hexapod_interfaces::msg::PointArray transformed_leg_positions;
        transformed_leg_positions.points.resize(6);
        transformed_leg_positions.points[0].x = leg_positions[0].x += 0.109;
        transformed_leg_positions.points[0].y = leg_positions[0].y += 0.189;
        transformed_leg_positions.points[0].z = leg_positions[0].z -= 0.082;

        transformed_leg_positions.points[1].x = leg_positions[1].x += 0.219;
        transformed_leg_positions.points[1].y = leg_positions[1].y += 0.0;
        transformed_leg_positions.points[1].z = leg_positions[1].z -= 0.082;

        transformed_leg_positions.points[2].x = leg_positions[2].x += 0.109;
        transformed_leg_positions.points[2].y = leg_positions[2].y -= 0.189;
        transformed_leg_positions.points[2].z = leg_positions[2].z -= 0.082;

        transformed_leg_positions.points[3].x = leg_positions[3].x -= 0.109;
        transformed_leg_positions.points[3].y = leg_positions[3].y -= 0.189;
        transformed_leg_positions.points[3].z = leg_positions[3].z -= 0.082;

        transformed_leg_positions.points[4].x = leg_positions[4].x -= 0.219;
        transformed_leg_positions.points[4].y = leg_positions[4].y += 0.0;
        transformed_leg_positions.points[4].z = leg_positions[4].z -= 0.082;

        transformed_leg_positions.points[5].x = leg_positions[5].x -= 0.109;
        transformed_leg_positions.points[5].y = leg_positions[5].y += 0.189;
        transformed_leg_positions.points[5].z = leg_positions[5].z -= 0.082;
                // usleep(100000);

        // RCLCPP_INFO_STREAM(this->get_logger(), "finished transform_leg_position");

        return transformed_leg_positions;
    }

    std::vector<geometry_msgs::msg::Point> leg_positions{geometry_msgs::msg::Point(), geometry_msgs::msg::Point(), 
                                                     geometry_msgs::msg::Point(), geometry_msgs::msg::Point(), 
                                                     geometry_msgs::msg::Point(), geometry_msgs::msg::Point()};
    int t_global=0;

    double velocity[3] = {0,0,0};

    double publish_frequency = 180;
    double total_control_points = 360;


    int gaits[4] = {0,1,2,3}; // 0 = tripod, 1 = wave, 2 = ripple, 3 = amble
    int gait = gaits[0];

    std::vector<std::pair<int,double>> step_swing_ratios {{0,2.0},{1,6.0},{2,3.0},{3,3.0}};
    double step_swing_ratio = step_swing_ratios[gait].second;

    std::vector<std::pair<int,double>> num_stance_control_points_vec {{0,total_control_points/2.0},{1,total_control_points*5.0/6.0},{2,total_control_points*2.0/3.0},{3,total_control_points*2.0/3.0}};
    double num_stance_control_points = num_stance_control_points_vec[gait].second;

    std::vector<std::pair<int,double>> num_swing_control_points_vec {{0,total_control_points/2.0},{1,total_control_points/6.0},{2,total_control_points/3.0},{3,total_control_points/3.0}};
    double num_swing_control_points = num_swing_control_points_vec[gait].second;

    std::vector<std::pair<int,double>> offsets {{0,total_control_points/2.0},{1,total_control_points/6.0},{2,total_control_points/6.0},{3,total_control_points/3.0}};
    double offset_ = offsets[gait].second;

    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point control;
    geometry_msgs::msg::Point end;
    double max_step_height = 0.05, max_step_length = 0.02;


    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joystick;
    rclcpp::Publisher<hexapod_interfaces::msg::PointArray>::SharedPtr publisher_;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> publisher_point_stamped;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodTrajectoryGenNode>());
    rclcpp::shutdown();
    return 0;
}
