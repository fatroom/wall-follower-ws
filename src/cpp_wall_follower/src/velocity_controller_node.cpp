#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

struct ControllerParams
{
  double kp;
  double max_speed;
  double target_distance;
  double watchdog_timeout;
};


class VelocityControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  VelocityControllerNode()
  : LifecycleNode("velocity_controller_node"),
    current_distance_(0.0)
  {
    this->declare_parameter<double>("target_distance", 1.0);
    this->declare_parameter<double>("kp", 0.5);
    this->declare_parameter<double>("max_speed", 0.5);
    this->declare_parameter<double>("watchdog_timeout", 1.0);

    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return this->parameters_callback(params);
    });
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double watchdog_timeout_ = 1.0; // seconds
  bool watchdog_triggered_ = false;

  std::optional<rclcpp::Time> last_msg_time_;
  double current_distance_;

  double target_distance_;
  double kp_;
  double max_speed_;

  void update_controller_params(const ControllerParams & params)
  {
    kp_ = params.kp;
    max_speed_ = params.max_speed;
    target_distance_ = params.target_distance;
    watchdog_timeout_ = params.watchdog_timeout;
  }

  bool validate_params(
    const ControllerParams & params,
    std::string & reason)
  {
    if (params.kp <= 0.0) {
      reason = "kp must be > 0";
      return false;
    }

    if (params.max_speed <= 0.0) {
      reason = "max_speed must be > 0";
      return false;
    }

    if (params.target_distance < 0.0) {
      reason = "target_distance must be >= 0";
      return false;
    }

    if (params.watchdog_timeout <= 0.0) {
      reason = "watchdog_timeout must be > 0";
      return false;
    }

    return true;
  }

  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & params)
  {
    ControllerParams new_params {
      kp_,
      max_speed_,
      target_distance_,
      watchdog_timeout_
    };

    for (const auto & param : params) {
      if (param.get_name() == "kp") {
        new_params.kp = param.as_double();
      } else if (param.get_name() == "max_speed") {
        new_params.max_speed = param.as_double();
      } else if (param.get_name() == "target_distance") {
        new_params.target_distance = param.as_double();
      } else if (param.get_name() == "watchdog_timeout") {
        new_params.watchdog_timeout = param.as_double();
      }
    }

    std::string reason;
    rcl_interfaces::msg::SetParametersResult result;

    if (!validate_params(new_params, reason)) {
      result.successful = false;
      result.reason = reason;
      return result;
    }

    update_controller_params(new_params);

    result.successful = true;
    return result;
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring from state %s", previous_state.label().c_str());

    ControllerParams params;
    params.kp = this->get_parameter("kp").as_double();
    params.max_speed = this->get_parameter("max_speed").as_double();
    params.target_distance = this->get_parameter("target_distance").as_double();
    params.watchdog_timeout = this->get_parameter("watchdog_timeout").as_double();

    std::string reason;

    if (!validate_params(params, reason)) {
      RCLCPP_ERROR(this->get_logger(), "%s", reason.c_str());
      return CallbackReturn::FAILURE;
    }

    update_controller_params(params);


    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", pub_qos);


    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating from state %s", previous_state.label().c_str());
    publisher_->on_activate();
    timer_ = this->create_wall_timer(100ms, [this]() {this->control_loop();});
    auto sub_qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/filtered_distance", sub_qos, [this](std_msgs::msg::Float32::UniquePtr msg) {
        current_distance_ = msg->data;
        last_msg_time_ = this->now();
        watchdog_triggered_ = false;
            });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating from state %s", previous_state.label().c_str());
    publish_zero();
    publisher_->on_deactivate();
    timer_.reset();
    subscription_.reset();

    last_msg_time_.reset();
    watchdog_triggered_ = false;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up from state %s", previous_state.label().c_str());
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  void control_loop()
  {
    if (!last_msg_time_.has_value()) {
      publish_zero();
      return;
    }

    auto now = this->now();

    if ((now - *last_msg_time_).seconds() > watchdog_timeout_) {
      if (!watchdog_triggered_) {
        RCLCPP_WARN(this->get_logger(),
          "Watchdog triggered: no distance messages received for %.2f seconds", watchdog_timeout_);
        watchdog_triggered_ = true;
      }
      publish_zero();
      return;
    }
    double error = target_distance_ - current_distance_;
    double velocity = kp_ * error;
    velocity = std::clamp(velocity, -max_speed_, max_speed_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    cmd_vel.angular.z = 0.0;
    publisher_->publish(cmd_vel);
  }

  void publish_zero()
  {
    if (!publisher_ || !publisher_->is_activated()) {
      return;
    }
    geometry_msgs::msg::Twist cmd_vel{};
    publisher_->publish(cmd_vel);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VelocityControllerNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
