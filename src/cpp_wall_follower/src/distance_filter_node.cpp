#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>

namespace cpp_wall_follower
{
class DistanceFilterNode : public rclcpp::Node {
public:
  DistanceFilterNode()
  : Node("distance_filter_node")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Smoothing factor for the low-pass filter";
    param_desc.additional_constraints = "0 < alpha < 1";
    this->declare_parameter<float>("alpha", 0.2, param_desc);

    param_callback_handle_ =
      this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          for (const auto & param : params) {
            if (param.get_name() == "alpha") {
              if (!update_alpha(param.get_value<float>())) {
                result.successful = false;
                result.reason = "Invalid alpha value. Must be between 0 and 1.";
              }
            }
          }
          return result;
        });

    auto sensorQoS = rclcpp::SensorDataQoS();
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/filtered_distance", sensorQoS);
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/raw_distance", sensorQoS, [this](std_msgs::msg::Float32::UniquePtr msg) {
        float distance = process_raw_distance(msg->data);
        publish_filtered_distance(distance);
            });
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

  float previous_distance_ = std::numeric_limits<float>::quiet_NaN();
  float alpha_ = 0.2;
  mutable std::mutex mutex_;

  bool update_alpha(float new_alpha)
  {
    std::scoped_lock lock(mutex_);
    if (new_alpha <= 0.0 || new_alpha >= 1.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid alpha value: %f. Must be between 0 and 1.",
        new_alpha);
      return false;
    }
    alpha_ = new_alpha;
    RCLCPP_INFO(this->get_logger(), "Updated alpha to: %f", alpha_);
    return true;
  }

  float process_raw_distance(float distance)
  {
    std::scoped_lock lock(mutex_);
    if (std::isnan(previous_distance_)) {previous_distance_ = distance;}
    float filtered_distance = alpha_ * distance + (1 - alpha_) * previous_distance_;
    previous_distance_ = filtered_distance;
    return filtered_distance;
  }

  void publish_filtered_distance(float distance)
  {
    auto message = std_msgs::msg::Float32();
    message.data = distance;
    RCLCPP_DEBUG(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }
};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_wall_follower::DistanceFilterNode>());
  rclcpp::shutdown();
  return 0;
}
