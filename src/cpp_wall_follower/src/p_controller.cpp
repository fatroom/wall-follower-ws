#include "cpp_wall_follower/p_controller.hpp"
#include <algorithm>
#include <atomic>

namespace cpp_wall_follower
{
PController::PController(const ControllerParams & params)
{
  auto p = std::make_shared<ControllerParams>(params);
  std::atomic_store(
      &params_,
      std::shared_ptr<const ControllerParams>(p));

  auto s = std::make_shared<MeasurementState>();
  std::atomic_store(
      &state_,
      std::shared_ptr<const MeasurementState>(s));
}

void PController::set_params(const ControllerParams & params)
{
  auto p = std::make_shared<ControllerParams>(params);
  std::atomic_store(
      &params_,
      std::shared_ptr<const ControllerParams>(p));
}

void PController::update_measurement(double distance, double time_sec)
{
  auto s = std::make_shared<MeasurementState>();
  s->distance = distance;
  s->last_msg_time = time_sec;
  s->has_measurement = true;

  std::atomic_store(
      &state_,
      std::shared_ptr<const MeasurementState>(s));
}

double PController::compute(double time_sec) const
{
  auto params = std::atomic_load(&params_);
  auto state = std::atomic_load(&state_);

  if (!state->has_measurement) {
    return 0.0;
  }

  if ((time_sec - state->last_msg_time) > params->watchdog_timeout) {
    return 0.0;
  }

  double error = params->target_distance - state->distance;

  double velocity = params->kp * error;
  return std::clamp(velocity, -params->max_speed, params->max_speed);
}

}
