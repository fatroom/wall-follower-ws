#include <gtest/gtest.h>
#include "wall_follower/p_distance_controller.hpp"

using namespace wall_follower;

TEST(PDistanceControllerTest, BasicProportionalResponse)
{
  ControllerParams params;
  params.kp = 1.0;
  params.max_speed = 2.0;
  params.target_distance = 5.0;
  params.watchdog_timeout = 1.0;
  params.deadband = 0.0;

  PDistanceController controller(params);

  controller.update_measurement(3.0, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 2.0);  // error = 2
}

TEST(PDistanceControllerTest, Saturation)
{
  ControllerParams params;
  params.kp = 10.0;
  params.max_speed = 1.0;
  params.target_distance = 10.0;
  params.watchdog_timeout = 1.0;
  params.deadband = 0.0;
  PDistanceController controller(params);

  controller.update_measurement(0.0, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 1.0);  // clamped
}

TEST(PDistanceControllerTest, Deadband)
{
  ControllerParams params;
  params.kp = 1.0;
  params.deadband = 0.1;
  params.target_distance = 5.0;
  params.watchdog_timeout = 1.0;

  PDistanceController controller(params);

  controller.update_measurement(4.95, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 0.0);
}

TEST(PDistanceControllerTest, WatchdogTimeout)
{
  ControllerParams params;
  params.watchdog_timeout = 0.5;

  PDistanceController controller(params);

  controller.update_measurement(1.0, 0.0);

  double cmd = controller.compute(1.0);

  EXPECT_DOUBLE_EQ(cmd, 0.0);  // timeout
}

TEST(PDistanceControllerTest, NoMeasurement)
{
  ControllerParams params{};

  PDistanceController controller(params);

  double cmd = controller.compute(0.0);

  EXPECT_DOUBLE_EQ(cmd, 0.0);
}
