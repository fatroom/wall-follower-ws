#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "cpp_wall_follower/distance_controller.hpp"
#include "cpp_wall_follower/p_distance_controller.hpp"


static constexpr size_t DEFAULT_EXECUTOR_THREADS = 4;

using namespace std::chrono_literals;


namespace cpp_wall_follower
{
class VelocityControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  VelocityControllerNode()
  : LifecycleNode("velocity_controller_node"),
    current_params_(),
    controller_(std::make_unique<cpp_wall_follower::PDistanceController>(
      cpp_wall_follower::ControllerParams{}))
  {
    // Declare parameters with descriptors and ranges
    auto target_distance_desc = rcl_interfaces::msg::ParameterDescriptor{};
    target_distance_desc.description = "Target distance from wall in meters";
    target_distance_desc.floating_point_range.resize(1);
    target_distance_desc.floating_point_range[0].from_value = 0.0;
    target_distance_desc.floating_point_range[0].to_value = 10.0;
    target_distance_desc.floating_point_range[0].step = 0.0;
    this->declare_parameter<double>("target_distance", 1.0, target_distance_desc);

    auto kp_desc = rcl_interfaces::msg::ParameterDescriptor{};
    kp_desc.description = "Proportional gain for distance error";
    kp_desc.floating_point_range.resize(1);
    kp_desc.floating_point_range[0].from_value = 0.01;
    kp_desc.floating_point_range[0].to_value = 10.0;
    kp_desc.floating_point_range[0].step = 0.0;
    kp_desc.additional_constraints = "Must be > 0";
    this->declare_parameter<double>("kp", 0.5, kp_desc);

    auto max_speed_desc = rcl_interfaces::msg::ParameterDescriptor{};
    max_speed_desc.description = "Maximum velocity command in m/s";
    max_speed_desc.floating_point_range.resize(1);
    max_speed_desc.floating_point_range[0].from_value = 0.01;
    max_speed_desc.floating_point_range[0].to_value = 5.0;
    max_speed_desc.floating_point_range[0].step = 0.0;
    max_speed_desc.additional_constraints = "Must be > 0";
    this->declare_parameter<double>("max_speed", 0.5, max_speed_desc);

    auto watchdog_timeout_desc = rcl_interfaces::msg::ParameterDescriptor{};
    watchdog_timeout_desc.description =
      "Maximum time in seconds without measurement before stopping";
    watchdog_timeout_desc.floating_point_range.resize(1);
    watchdog_timeout_desc.floating_point_range[0].from_value = 0.01;
    watchdog_timeout_desc.floating_point_range[0].to_value = 10.0;
    watchdog_timeout_desc.floating_point_range[0].step = 0.0;
    watchdog_timeout_desc.additional_constraints = "Must be > 0";
    this->declare_parameter<double>("watchdog_timeout", 1.0, watchdog_timeout_desc);

    auto deadband_desc = rcl_interfaces::msg::ParameterDescriptor{};
    deadband_desc.description = "Error deadband in meters to prevent oscillations near target";
    deadband_desc.floating_point_range.resize(1);
    deadband_desc.floating_point_range[0].from_value = 0.0;
    deadband_desc.floating_point_range[0].to_value = 1.0;
    deadband_desc.floating_point_range[0].step = 0.0;
    this->declare_parameter<double>("deadband", 0.02, deadband_desc);

    // Initialize current_params_ from declared parameters
    current_params_ = load_params_from_ros();

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

  cpp_wall_follower::ControllerParams current_params_;
  std::unique_ptr<cpp_wall_follower::DistanceController> controller_;

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
    // Start from current committed state (single source of truth)
    auto new_params = current_params_;

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

    // Commit to current_params_ (single source of truth)
    current_params_ = new_params;

    // Update controller with committed params
    controller_->set_params(current_params_);

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

    // Load from ROS parameters
    auto params = load_params_from_ros();

    std::string reason;
    if (!validate_params(params, reason)) {
      RCLCPP_ERROR(this->get_logger(), "%s", reason.c_str());
      return CallbackReturn::FAILURE;
    }

    // Commit to current_params_ (single source of truth)
    current_params_ = params;

    // Update controller with committed params
    controller_->set_params(current_params_);

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
        controller_->update_measurement(msg->data, this->now().seconds());
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
    double velocity = controller_->compute(now_sec);

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
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cpp_wall_follower::VelocityControllerNode>();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), DEFAULT_EXECUTOR_THREADS
  );
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
