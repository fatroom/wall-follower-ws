#pragma once

#include "cpp_wall_follower/distance_controller.hpp"

namespace cpp_wall_follower
{
struct MeasurementState
{
  double distance{0.0};
  double last_msg_time{0.0};
  bool has_measurement{false};
};

class PDistanceController : public DistanceController
{
public:
  explicit PDistanceController(const ControllerParams & params);

  void set_params(const ControllerParams & params) override;
  void update_measurement(double distance, double time_sec) override;

  double compute(double time_sec) const override;

private:
  ControllerParams params_;
  MeasurementState state_;
};
}
