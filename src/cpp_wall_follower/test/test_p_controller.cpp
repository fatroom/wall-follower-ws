#include <gtest/gtest.h>
#include "cpp_wall_follower/p_controller.hpp"

using namespace cpp_wall_follower;

TEST(PControllerTest, BasicProportionalResponse)
{
  ControllerParams params;
  params.kp = 1.0;
  params.max_speed = 2.0;
  params.target_distance = 5.0;
  params.watchdog_timeout = 1.0;

  PController controller(params);

  controller.update_measurement(3.0, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 2.0);  // error = 2
}

TEST(PControllerTest, Saturation)
{
  ControllerParams params;
  params.kp = 10.0;
  params.max_speed = 1.0;
  params.target_distance = 10.0;

  PController controller(params);

  controller.update_measurement(0.0, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 0);  // clamped
}

TEST(PControllerTest, Deadband)
{
  ControllerParams params;
  params.kp = 1.0;
  //params.deadband = 0.1;
  params.target_distance = 5.0;

  PController controller(params);

  controller.update_measurement(4.95, 0.0);

  double cmd = controller.compute(0.1);

  EXPECT_DOUBLE_EQ(cmd, 0.0);
}

TEST(PControllerTest, WatchdogTimeout)
{
  ControllerParams params;
  params.watchdog_timeout = 0.5;

  PController controller(params);

  controller.update_measurement(1.0, 0.0);

  double cmd = controller.compute(1.0);

  EXPECT_DOUBLE_EQ(cmd, 0.0);  // timeout
}

TEST(PControllerTest, NoMeasurement)
{
  ControllerParams params{};

  PController controller(params);

  double cmd = controller.compute(0.0);

  EXPECT_DOUBLE_EQ(cmd, 0.0);
}
