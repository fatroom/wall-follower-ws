#include "cpp_wall_follower/p_controller.hpp"
#include <algorithm>

namespace cpp_wall_follower
{
PController::PController(const ControllerParams & params)
: params_(params), state_()
{
}

void PController::set_params(const ControllerParams & params)
{
  params_ = params;
}

void PController::update_measurement(double distance, double time_sec)
{
  state_.distance = distance;
  state_.last_msg_time = time_sec;
  state_.has_measurement = true;
}

double PController::compute(double time_sec) const
{
  if (!state_.has_measurement) {
    return 0.0;
  }

  if ((time_sec - state_.last_msg_time) > params_.watchdog_timeout) {
    return 0.0;
  }

  double error = params_.target_distance - state_.distance;

  if (std::abs(error) < params_.deadband) {
    return 0.0;
  }

  double velocity = params_.kp * error;
  return std::clamp(velocity, -params_.max_speed, params_.max_speed);
}

}
