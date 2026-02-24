#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


class DistanceFilterNode: public rclcpp::Node {
public:
    DistanceFilterNode() : Node("distance_filter_node") 
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Smoothing factor for the low-pass filter (0 < alpha < 1)";
        this -> declare_parameter<float>("alpha", 0.2, param_desc);

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/filtered_distance", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/raw_distance", 10, [this](std_msgs::msg::Float32::UniquePtr msg) {
                float distance = process_raw_distance(msg->data);
                publish_filtered_distance(distance);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    float process_raw_distance(float distance) {
        static float previous_distance = distance;
        float alpha = this -> get_parameter("alpha").get_value<float>();
        float filtered_distance = alpha * distance + (1 - alpha) * previous_distance;
        previous_distance = filtered_distance;
        return filtered_distance;
    }

    void publish_filtered_distance(float distance) {
        auto message = std_msgs::msg::Float32();
        message.data = distance;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceFilterNode>());
    rclcpp::shutdown();
    return 0;
}