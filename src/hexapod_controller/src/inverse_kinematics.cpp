#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array>
#include <vector>
#include <cmath>

using namespace std::placeholders;

class HexapodIKNode : public rclcpp::Node {
public:
    HexapodIKNode() : Node("hexapod_ik_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tip_poses", 10, std::bind(&HexapodIKNode::tipPosesCallback, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_position_controller/commands", 10);

    }

private:
    void tipPosesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if (msg->data.size() != 18) {
            RCLCPP_WARN(this->get_logger(), "Invalid data size in /tip_poses, expected 18 values");
            return;
        }

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.resize(18);

        for(int i=0 ; i<6 ; i++){
            double X = msg->data[3*i];
            double Y = msg->data[3*i+1];
            double Z = msg->data[3*i+2];

            double J1;
            double J2;
            double J3;
            
            if(std::sqrt(X*X) + std::sqrt(Y*Y) + std::sqrt(Z*Z) > femur_length + tibia_length){
    //            // RCLCPP_INFO(this->get_logger(),"POINT OUT oF BOUNDS");
            }
            else{
                double J1L = coxa_length;
                double J2L = femur_length;
                double J3L = tibia_length;
                // CALCULATE INVERSE KINEMATIC SOLUTION
                J1 = atan(X / Y);
                double H = sqrt((Y * Y) + (X * X));
                double L = sqrt((H * H) + (Z * Z));
                J3 = acos(   ((J2L * J2L) + (J3L * J3L) - (L * L))   /   (2 * J2L * J3L)   );
                double B = acos(   ((L * L) + (J2L * J2L) - (J3L * J3L))   /   (2 * L * J2L)   );
                double A = atan(Z / H);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
                J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

                //model correction
                J1 = std::min(std::max((60.0*3.1415/180.0 - J1),0.0),2.0944);
                J2 = std::min(std::max((90.0*3.1415/180.0 - J2),-0.785398),1.5708);
                J3 = std::min(std::max(J3,0.0),2.0944);

                cmd.data[i*3] = J1;
                cmd.data[i*3+1] = J2;
                cmd.data[i*3+2] = J3;
               
            }
    //        // RCLCPP_INFO(this->get_logger(),"LEG: %d :xyz (%.3f, %.3f, %.3f) J1 %.2f J2 %.2f J3 %.2f",i+1,X,Y,Z, J1, J2, J3);

            publisher_->publish(cmd);
        }   
    }

    float coxa_length = 0, femur_length = 0.08, tibia_length = 0.177;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodIKNode>());
    rclcpp::shutdown();
    return 0;
}
