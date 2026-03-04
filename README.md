# Wall Follower

A ROS 2 wall-following robot demo built in C++. 
A proportional controller maintains a target distance from a wall using a simulated distance sensor, a low-pass filter, and a lifecycle-managed velocity controller.

## Architecture

```
distance_sensor_node → /raw_distance → distance_filter_node → /filtered_distance → wall_follower_node → /cmd_vel
```

- **distance_sensor_node** — Publishes simulated noisy distance readings at 20 Hz.
- **distance_filter_node** — Applies an exponential moving average (configurable `alpha`) to smooth sensor noise.
- **wall_follower_node** — Lifecycle-managed node that runs a P-controller to produce `geometry_msgs/Twist` commands. Includes a watchdog timeout and deadband.

## Requirements

- ROS 2 (tested with Jazzy)
- `rclcpp`, `rclcpp_lifecycle`, `std_msgs`, `geometry_msgs`

## Build

```bash
colcon build --packages-select wall_follower
source install/setup.bash
```

## Run

```bash
ros2 launch wall_follower wall_follower.launch.py
```

The launch file starts all three nodes and automatically transitions the controller through configure → activate.

## Parameters

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| distance_filter_node | `alpha` | 0.2 | Smoothing factor (0–1, lower = smoother) |
| wall_follower_node | `target_distance` | 1.0 | Desired wall distance (m) |
| wall_follower_node | `kp` | 0.5 | Proportional gain |
| wall_follower_node | `max_speed` | 0.5 | Velocity clamp (m/s) |
| wall_follower_node | `watchdog_timeout` | 1.0 | Seconds without data before stopping |
| wall_follower_node | `deadband` | 0.02 | Error threshold to suppress oscillation (m) |

## Tests

```bash
colcon test --packages-select wall_follower
colcon test-result --verbose
```

Includes unit tests for the P-controller and integration tests for the full pipeline and lifecycle transitions.

## License

MIT
