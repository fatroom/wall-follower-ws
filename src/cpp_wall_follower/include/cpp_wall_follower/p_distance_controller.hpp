#pragma once

#include "cpp_wall_follower/distance_controller.hpp"
#include <mutex>

namespace cpp_wall_follower
{
/**
 * @brief Proportional controller for maintaining target distance from a wall
 *
 * This controller implements a P-controller (proportional control) to compute
 * velocity commands that drive a robot to maintain a target distance from a wall.
 *
 * Features:
 * - Proportional control with configurable gain (kp)
 * - Output saturation to respect maximum speed limits
 * - Deadband to prevent oscillation near target distance
 * - Watchdog timeout to stop robot if measurements become stale
 *
 * Thread Safety:
 * All public methods are thread-safe and can be called concurrently from
 * multiple threads. Internal state is protected by a mutex.
 */
class PDistanceController : public DistanceController
{
public:
  /**
   * @brief Construct controller with given parameters
   * @param params Initial controller parameters including gains, limits, and timeouts
   * @note No locking is needed during construction
   */
  explicit PDistanceController(const ControllerParams & params);

  /**
   * @brief Update controller parameters atomically
   * @param params New controller parameters to apply
   * @note Thread-safe: Can be called concurrently with compute() and update_measurement()
   * @note Changes take effect immediately for subsequent compute() calls
   */
  void set_params(const ControllerParams & params) override;

  /**
   * @brief Update controller with new distance measurement
   * @param distance Measured distance to wall in meters
   * @param time_sec Timestamp of measurement in seconds since epoch
   * @note Thread-safe: Can be called concurrently with compute() and set_params()
   * @note Sets last_msg_time for watchdog
   */
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
