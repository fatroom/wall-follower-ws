#pragma once

namespace cpp_wall_follower
{
struct ControllerParams
{
  double kp{0.0};
  double ki{0.0};          // future use
  double kd{0.0};          // future use
  double max_speed{0.0};
  double target_distance{0.0};
  double watchdog_timeout{0.0};
  double deadband{0.0};
};

class DistanceController
{
public:
  virtual ~DistanceController() = default;

  virtual void set_params(const ControllerParams & params) = 0;
  virtual void update_measurement(double distance, double time_sec) = 0;
  virtual double compute(double time_sec) const = 0;
};
}
