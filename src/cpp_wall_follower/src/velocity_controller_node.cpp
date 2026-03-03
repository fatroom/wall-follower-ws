#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "cpp_wall_follower/p_controller.hpp"


using namespace std::chrono_literals;


class VelocityControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  VelocityControllerNode()
  : LifecycleNode("velocity_controller_node"),
    controller_(cpp_wall_follower::ControllerParams{})
  {
    this->declare_parameter<double>("target_distance", 1.0);
    this->declare_parameter<double>("kp", 0.5);
    this->declare_parameter<double>("max_speed", 0.5);
    this->declare_parameter<double>("watchdog_timeout", 1.0);
    this->declare_parameter<double>("deadband", 0.02);

    callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return this->parameters_callback(params);
    });
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  cpp_wall_follower::PController controller_;

  bool validate_params(
    const cpp_wall_follower::ControllerParams & params,
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

    if (params.deadband < 0.0) {
      reason = "deadband must be >= 0";
      return false;
    }

    return true;
  }

  cpp_wall_follower::ControllerParams load_params_from_ros() const
  {
    cpp_wall_follower::ControllerParams params;
    params.kp = this->get_parameter("kp").as_double();
    params.max_speed = this->get_parameter("max_speed").as_double();
    params.target_distance = this->get_parameter("target_distance").as_double();
    params.watchdog_timeout = this->get_parameter("watchdog_timeout").as_double();
    params.deadband = this->get_parameter("deadband").as_double();
    return params;
  }

  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & params)
  {
    // Load current parameter values
    auto new_params = load_params_from_ros();

    // Apply the proposed parameter changes
    for (const auto & param : params) {
      if (param.get_name() == "kp") {
        new_params.kp = param.as_double();
      } else if (param.get_name() == "max_speed") {
        new_params.max_speed = param.as_double();
      } else if (param.get_name() == "target_distance") {
        new_params.target_distance = param.as_double();
      } else if (param.get_name() == "watchdog_timeout") {
        new_params.watchdog_timeout = param.as_double();
      } else if (param.get_name() == "deadband") {
        new_params.deadband = param.as_double();
      }
    }

    std::string reason;
    rcl_interfaces::msg::SetParametersResult result;

    if (!validate_params(new_params, reason)) {
      result.successful = false;
      result.reason = reason;
      return result;
    }

    controller_.set_params(new_params);

    RCLCPP_INFO_STREAM(this->get_logger(),
      "Updated params: kp=" << new_params.kp
                            << " max_speed=" << new_params.max_speed
                            << " target=" << new_params.target_distance
                            << " watchdog=" << new_params.watchdog_timeout
                            << " deadband=" << new_params.deadband);

    result.successful = true;
    return result;
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring from state %s", previous_state.label().c_str());

    auto params = load_params_from_ros();

    std::string reason;

    if (!validate_params(params, reason)) {
      RCLCPP_ERROR(this->get_logger(), "%s", reason.c_str());
      return CallbackReturn::FAILURE;
    }

    controller_.set_params(params);

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", pub_qos);


    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating from state %s", previous_state.label().c_str());
    publisher_->on_activate();
    timer_ = this->create_wall_timer(
      100ms,
      [this]() {this->control_loop();},
      callback_group_);
    auto sub_qos = rclcpp::SensorDataQoS();
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_;
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/filtered_distance",
      sub_qos,
      [this](std_msgs::msg::Float32::UniquePtr msg) {
        controller_.update_measurement(msg->data, this->now().seconds());
      },
      sub_options);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating from state %s", previous_state.label().c_str());
    publish_zero();
    publisher_->on_deactivate();
    timer_.reset();
    subscription_.reset();

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
    if (!publisher_ || !publisher_->is_activated()) {
      return;
    }

    double now_sec = this->now().seconds();
    double velocity = controller_.compute(now_sec);

    geometry_msgs::msg::Twist cmd_vel{};
    cmd_vel.linear.x = velocity;

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

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    4  // number of threads
  );
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
