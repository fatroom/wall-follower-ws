#pragma once

#include <memory>

namespace cpp_wall_follower
{
struct ControllerParams
{
  double kp{0.5};
  double max_speed{0.5};
  double target_distance{1.0};
  double watchdog_timeout{1.0};
  double deadband{0.02};
};

struct MeasurementState
{
  double distance{0.0};
  double last_msg_time{0.0};
  bool has_measurement{false};
};

class PController
{
public:
  explicit PController(const ControllerParams & params);

  void set_params(const ControllerParams & params);
  void update_measurement(double distance, double time_sec);

  double compute(double time_sec) const;

private:
  std::shared_ptr<const ControllerParams> params_;
  std::shared_ptr<const MeasurementState> state_;
};
}
