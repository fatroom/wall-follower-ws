#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;


class VelocityControllerNode : public rclcpp::Node {
public:
  VelocityControllerNode()
  : Node("velocity_controller_node")
  {
    this->declare_parameter<double>("target_distance", 1.0);
    this->declare_parameter<double>("kp", 0.5);
    this->declare_parameter<double>("max_speed", 0.5);

    target_distance_ = this->get_parameter("target_distance").as_double();
    kp_ = this->get_parameter("kp").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/filtered_distance", 10, [this](std_msgs::msg::Float32::UniquePtr msg) {
        current_distance_ = msg->data;
            });

    timer_ = this->create_wall_timer(100ms, [this]() {this->control_loop();});
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<double> current_distance_;

  double target_distance_;
  double kp_;
  double max_speed_;

  void control_loop()
  {
    if (!current_distance_.has_value()) {
      return;
    }

    double error = target_distance_ - current_distance_.value();
    double velocity = kp_ * error;
    velocity = std::clamp(velocity, -max_speed_, max_speed_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    cmd_vel.angular.z = 0.0;
    publisher_->publish(cmd_vel);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
