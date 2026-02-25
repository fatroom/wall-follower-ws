#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <random>

using namespace std::chrono_literals;

class DistanceSensorNode : public rclcpp::Node {
public:
  DistanceSensorNode()
  :Node("distance_sensor_node"),
    generator_(std::random_device{}()),
    noise_(0, 0.05)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/raw_distance", 10);
    timer_ = this->create_wall_timer(50ms, [this]() {this->publish();});
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::random_device rd_;
  std::mt19937 generator_;
  std::normal_distribution<float> noise_;

  void publish()
  {
    auto message = std_msgs::msg::Float32();
    message.data = get_distance_measurement();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }

  float get_distance_measurement()
  {
        // Simulate a varying distance measurement
    float true_distance = 1.0 + 0.5 * sin(this->now().nanoseconds());
        // Add some random noise to the measurement
    float reading = true_distance + noise_(generator_);
    return reading;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceSensorNode>());
  rclcpp::shutdown();
  return 0;
}
