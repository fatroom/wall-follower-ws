#pragma once

#include "cpp_wall_follower/distance_controller.hpp"
#include <mutex>

namespace cpp_wall_follower
{
class PDistanceController : public DistanceController
{
public:
  explicit PDistanceController(const ControllerParams & params);

  void set_params(const ControllerParams & params) override;
  void update_measurement(double distance, double time_sec) override;

  /**
   * @brief Compute control output based on current state
   * @param time_sec Current time in seconds for watchdog comparison
   * @return Velocity command in m/s, clamped to [-max_speed, max_speed]
   * @note Returns 0.0 if no measurement, watchdog timeout, or within deadband
   */
  double compute(double time_sec) const override;

private:
  struct State
  {
    double distance{0.0};
    double last_msg_time{0.0};
    bool has_measurement{false};
  };

  mutable std::mutex mutex_;

  ControllerParams params_;
  State state_;
};
}
