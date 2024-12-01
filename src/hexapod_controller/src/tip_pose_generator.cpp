#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// #include <array>
#include <vector>
// #include <utility>
#include <cmath>
#include "hexapod_interfaces/msg/point_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


using namespace std::placeholders;

struct Point3D {

    double x, y, z;
};

Point3D rotatePoint(const Point3D& point, double alpha, double beta, double gamma) {

    // // Convert angles from degrees to radians
    // alpha = toRadians(alpha);
    // beta = toRadians(beta);
    // gamma = toRadians(gamma);

    // Step 1: Rotate around the x-axis
    double x1 = point.x;
    double y1 = point.y * cos(alpha) - point.z * sin(alpha);
    double z1 = point.y * sin(alpha) + point.z * cos(alpha);

    // Step 2: Rotate around the y-axis
    double x2 = x1 * cos(beta) + z1 * sin(beta);
    double y2 = y1;
    double z2 = -x1 * sin(beta) + z1 * cos(beta);

    // Step 3: Rotate around the z-axis
    double x3 = x2 * cos(gamma) - y2 * sin(gamma);
    double y3 = x2 * sin(gamma) + y2 * cos(gamma);
    double z3 = z2;
    
    return {x3, y3, z3};
}

class HexapodTipPoseGenNode : public rclcpp::Node {
public:
    HexapodTipPoseGenNode() : Node("hexapod_tip_pose_generator") {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Reached constructor _1");

        this->leg_base_coordinates.points.resize(6);

        this->leg_base_coordinates.points[0].x = 0.0681;
        this->leg_base_coordinates.points[0].y = 0.1180;
        this->leg_base_coordinates.points[0].z = 0.0208;

        this->leg_base_coordinates.points[1].x = 0.1363;
        this->leg_base_coordinates.points[1].y = 0.0;
        this->leg_base_coordinates.points[1].z = 0.0208;

        this->leg_base_coordinates.points[2].x = 0.0681;
        this->leg_base_coordinates.points[2].y = -0.1180;
        this->leg_base_coordinates.points[2].z = 0.0208;

        this->leg_base_coordinates.points[3].x = -0.0681;
        this->leg_base_coordinates.points[3].y = -0.1180;
        this->leg_base_coordinates.points[3].z = 0.0208;

        this->leg_base_coordinates.points[4].x = -0.1363;
        this->leg_base_coordinates.points[4].y = 0.0;
        this->leg_base_coordinates.points[4].z = 0.0208;

        this->leg_base_coordinates.points[5].x = -0.0681;
        this->leg_base_coordinates.points[5].y = 0.1180;
        this->leg_base_coordinates.points[5].z = 0.0208;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Reached constructor _2");

        subscription_tip_trajectory = this->create_subscription<hexapod_interfaces::msg::PointArray>(
            "/trajectory_poses", 10, std::bind(&HexapodTipPoseGenNode::update_tip_points, this, _1));

        // RCLCPP_INFO_STREAM(this->get_logger(), "created subscription subscription_tip_trajectory");

        subscription_body_pose = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pid_pose_commands", 10, std::bind(&HexapodTipPoseGenNode::update_body_pose_data, this, _1));

        // RCLCPP_INFO_STREAM(this->get_logger(), "created subscription subscription_body_pose");

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tip_poses", 10);

        // RCLCPP_INFO_STREAM(this->get_logger(), "created publisher publisher_");

        /////////////////// rviz visualization //////////////////
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped0", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped1", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped2", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped3", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped4", 10));
        publisher_point_stamped.push_back(this->create_publisher<geometry_msgs::msg::PointStamped>("/final_point_stamped5", 10));

    }

private:

    void update_tip_points(const hexapod_interfaces::msg::PointArray& msg){
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback");
        hexapod_interfaces::msg::PointArray final_tip_poses_global;
        final_tip_poses_global.points.resize(6);
        // we shall proceed to apply various transformations to this set of points
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback _ 1");


        for (int i = 0; i < 6; ++i) {
            final_tip_poses_global.points[i].x = msg.points[i].x;
            final_tip_poses_global.points[i].y = msg.points[i].y;
            final_tip_poses_global.points[i].z = msg.points[i].z;
        }

        for(int i=0 ; i<6 ; i++){
            
            // translating the body/points (moving points of legs in minus will move the body in plus direction)
            final_tip_poses_global.points[i].x -= this->body_pose_xyz_received[0];
            final_tip_poses_global.points[i].y -= this->body_pose_xyz_received[1];
            final_tip_poses_global.points[i].z -= this->body_pose_xyz_received[2];

            

            // rotating the body/points
            Point3D point = {final_tip_poses_global.points[i].x, final_tip_poses_global.points[i].y, final_tip_poses_global.points[i].z};
            // RCLCPP_INFO(this->get_logger(),"[%d] before rotn body : (%.3f, %.3f, %.3f)",i,point.x,point.y,point.z);

            point = rotatePoint(point, this->body_pose_rpy_received[1], -(this->body_pose_rpy_received[0]), this->body_pose_rpy_received[2]);
            // RCLCPP_INFO(this->get_logger(),"[%d] after rotn body : (%.3f, %.3f, %.3f)",i, point.x,point.y,point.z);


            final_tip_poses_global.points[i].x = point.x;
            final_tip_poses_global.points[i].y = point.y;
            final_tip_poses_global.points[i].z = point.z;

            /////////////// rviz visualization ////////////////////////
            // for(int i=0; i<6 ; i++){
                geometry_msgs::msg::PointStamped leg_point;

                leg_point.header.frame_id = "body";  // Set the frame of reference
                leg_point.header.stamp = this->get_clock()->now();

                leg_point.point.x = final_tip_poses_global.points[i].y;
                leg_point.point.y = -final_tip_poses_global.points[i].x;
                leg_point.point.z = final_tip_poses_global.points[i].z;
                publisher_point_stamped[i]->publish(leg_point);
            // }
            

            // converting to local coordinates
            final_tip_poses_global.points[i].x -= this->leg_base_coordinates.points[i].x;
            final_tip_poses_global.points[i].y -= this->leg_base_coordinates.points[i].y;
            final_tip_poses_global.points[i].z -= this->leg_base_coordinates.points[i].z;

   //         // RCLCPP_INFO(this->get_logger(),"[%d] local coord before_rotn: (%.3f, %.3f, %.3f)",i,final_tip_poses_global.points[i].x,final_tip_poses_global.points[i].y,final_tip_poses_global.points[i].z);

        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback _ 2");

        Point3D point_leg[6];
        for(int i=0 ; i<6 ; i++){
            point_leg[i].x = final_tip_poses_global.points[i].x;
            point_leg[i].y = final_tip_poses_global.points[i].y;
            point_leg[i].z = final_tip_poses_global.points[i].z;
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback _ 3");

        for(int i=0 ; i< 6 ; i++){
            point_leg[i] = rotatePoint(point_leg[i], 0, 0, i*1.0472 + 0.523599);

            final_tip_poses_global.points[i].x = point_leg[i].x;
            final_tip_poses_global.points[i].y = point_leg[i].y;
            final_tip_poses_global.points[i].z = point_leg[i].z;
    //        // RCLCPP_INFO(this->get_logger(),"[%d] local coord after_rotn: (%.3f, %.3f, %.3f)",i,final_tip_poses_global.points[i].x,final_tip_poses_global.points[i].y,final_tip_poses_global.points[i].z);

        }


        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback _ 4");

        std_msgs::msg::Float64MultiArray final_tip_poses_local;
        final_tip_poses_local.data.resize(18);  // 6 points, each with 3 values (x, y, z)
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_tip_points callback _ 5");

        for(int i=0 ; i<6 ; i++){
            final_tip_poses_local.data[3 * i] = final_tip_poses_global.points[i].x;
            final_tip_poses_local.data[3*i+1] = final_tip_poses_global.points[i].y;
            final_tip_poses_local.data[3*i+2] = final_tip_poses_global.points[i].z;

        }

        publisher_->publish(final_tip_poses_local);
        // RCLCPP_INFO_STREAM(this->get_logger(), "exiting  update_tip_points callback");

    }

    void update_body_pose_data(const std_msgs::msg::Float64MultiArray& msg){
        // RCLCPP_INFO_STREAM(this->get_logger(), "entered  update_body_pose_data callback");

        if (msg.data.size() >= 6) {
            for (int i = 0; i < 3; i++) { 
                this->body_pose_rpy_received[i] = msg.data[i];
                this->body_pose_xyz_received[i] = msg.data[i + 3];
            }
            // RCLCPP_INFO(this->get_logger(),"rpy received (%.2f,%.2f,%.2f), xyz received (%.2f,%.2f,%.2f)", body_pose_rpy_received[0], body_pose_rpy_received[1], body_pose_rpy_received[2], body_pose_xyz_received[0], body_pose_xyz_received[1], body_pose_xyz_received[2]);
        }
    }

    std::vector<double> body_pose_xyz_received{0.0, 0.0, 0.0};
    std::vector<double> body_pose_rpy_received{0.0, 0.0, 0.0};

    hexapod_interfaces::msg::PointArray leg_base_coordinates;

    rclcpp::Subscription<hexapod_interfaces::msg::PointArray>::SharedPtr subscription_tip_trajectory;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_body_pose;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> publisher_point_stamped;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodTipPoseGenNode>());
    rclcpp::shutdown();
    return 0;
}
