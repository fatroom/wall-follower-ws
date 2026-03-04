#pragma once

namespace wall_follower
{
/**
 * @brief Controller configuration parameters
 *
 * Parameters controlling the behavior of distance controllers.
 */
struct ControllerParams
{
  double kp{0.0};                    ///< Proportional gain (P-term)
  double ki{0.0};                    ///< Integral gain (I-term) - reserved for future use
  double kd{0.0};                    ///< Derivative gain (D-term) - reserved for future use
  double max_speed{0.0};             ///< Maximum output velocity in m/s (saturation limit)
  double target_distance{0.0};       ///< Desired distance to wall in meters
  double watchdog_timeout{0.0};      ///< Maximum age of measurement in seconds before stopping
  double deadband{0.0};              ///< Distance error threshold in meters below which output is zero
};

/**
 * @brief Abstract base class for distance-based controllers
 *
 * Defines the interface contract for controllers that maintain a robot's
 * distance from a wall or obstacle. Implementations receive distance measurements
 * and compute velocity commands to achieve the target distance.
 *
 * Expected Usage Pattern:
 * 1. Construct controller with initial parameters
 * 2. Call update_measurement() when new sensor data arrives
 * 3. Call compute() periodically to generate velocity commands
 * 4. Call set_params() to adjust controller behavior at runtime
 */
class DistanceController
{
public:
  virtual ~DistanceController() = default;

  /**
   * @brief Update controller parameters
   * @param params New controller configuration
   */
  virtual void set_params(const ControllerParams & params) = 0;

  /**
   * @brief Provide new distance measurement to controller
   * @param distance Measured distance to wall in meters
   * @param time_sec Timestamp of measurement in seconds
   */
  virtual void update_measurement(double distance, double time_sec) = 0;

  /**
   * @brief Compute control output based on current state
   * @param time_sec Current time in seconds for timeout checking
   * @return Velocity command in m/s
   */
  virtual double compute(double time_sec) const = 0;
};
}
