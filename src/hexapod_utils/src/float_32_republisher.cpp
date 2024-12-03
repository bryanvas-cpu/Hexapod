#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::placeholders;


class RepublisherNode : public rclcpp::Node
{
public:
    RepublisherNode()
        : Node("pub_sub_node") // Node name
    {
        // Create a publisher on the "output_topic"
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/esp/int16array_sub", 10);

        // Create a subscription on the "input_topic"
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/simple_position_controller/commands", 10,
            std::bind(&RepublisherNode::callback, this, _1));
    }

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Create a new message to publish
        auto new_msg = std_msgs::msg::Int16MultiArray();
        new_msg.data.resize(18);

        // Cast each double to float
        for (size_t i = 0; i < 18; ++i) {
            new_msg.data[i] = (int16_t)(msg->data[i] * 4095.0/6.2831);
        }

        // Publish the new message
        publisher_->publish(new_msg);

        // Log the published message size for debugging (optional)
        RCLCPP_INFO(this->get_logger(), "Published a message of size: %ld", new_msg.data.size());
    }


    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node instance
    auto node = std::make_shared<RepublisherNode>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
