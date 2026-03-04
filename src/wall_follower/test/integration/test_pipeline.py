#!/usr/bin/env python3

import time
import unittest

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for test."""
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('wall_follower'),
                            'launch',
                            'wall_follower.launch.py',
                        ]
                    ),
                )
            ),
            ReadyToTest(),
        ]
    )


class TestPipeline(unittest.TestCase):
    """Integration test for sensor-filter-controller pipeline."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = Node('test_pipeline_node')
        self.raw_distances = []
        self.filtered_distances = []
        self.cmd_vels = []

        # Create subscriptions with matching QoS profiles
        # Use SensorDataQoS for raw and filtered distance to match publisher QoS
        self.raw_sub = self.node.create_subscription(
            Float32,
            'raw_distance',
            lambda msg: self.raw_distances.append(msg.data),
            qos_profile_sensor_data,
        )

        self.filtered_sub = self.node.create_subscription(
            Float32,
            'filtered_distance',
            lambda msg: self.filtered_distances.append(msg.data),
            qos_profile_sensor_data,
        )

        # cmd_vel uses default reliable QoS
        self.cmd_vel_sub = self.node.create_subscription(
            Twist, 'cmd_vel', lambda msg: self.cmd_vels.append(msg.linear.x), 10
        )

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def spin_for_messages(self, duration_sec, min_messages=10):
        """Spin node to collect messages."""
        start_time = time.time()
        while (time.time() - start_time) < duration_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                len(self.raw_distances) >= min_messages
                and len(self.filtered_distances) >= min_messages
                and len(self.cmd_vels) >= min_messages
            ):
                break

    def calculate_variance(self, samples):
        """Calculate variance of samples."""
        if len(samples) < 2:
            return 0.0
        mean = sum(samples) / len(samples)
        return sum((x - mean) ** 2 for x in samples) / len(samples)

    def test_message_flow(self):
        """Test that all topics publish messages."""
        # Collect messages for 3 seconds
        self.spin_for_messages(duration_sec=3.0, min_messages=10)

        # Verify messages received on all topics
        self.assertGreater(
            len(self.raw_distances),
            10,
            'Should receive at least 10 raw distance messages',
        )
        self.assertGreater(
            len(self.filtered_distances),
            10,
            'Should receive at least 10 filtered distance messages',
        )
        self.assertGreater(
            len(self.cmd_vels), 5, 'Should receive at least 5 cmd_vel messages'
        )

    def test_filtering_reduces_noise(self):
        """Test that filtered signal has lower variance than raw."""
        # Collect sufficient samples
        self.spin_for_messages(duration_sec=3.0, min_messages=50)

        # Calculate variances
        raw_variance = self.calculate_variance(self.raw_distances[:50])
        filtered_variance = self.calculate_variance(self.filtered_distances[:50])

        # Verify filtering reduces noise
        self.assertLess(
            filtered_variance,
            raw_variance,
            f'Filtered variance ({filtered_variance:.6f}) should be less than '
            f'raw variance ({raw_variance:.6f})',
        )

        # Verify reduction is significant (at least 20%)
        reduction_ratio = filtered_variance / raw_variance
        self.assertLess(
            reduction_ratio,
            0.8,
            'Filtering should reduce variance by at least 20%'
        )

    def test_controller_responds_to_distance(self):
        """Test that controller generates appropriate velocity commands."""
        # Collect messages
        self.spin_for_messages(duration_sec=3.0, min_messages=20)

        # Verify cmd_vel messages are not all zero (robot is responding)
        non_zero_cmds = [v for v in self.cmd_vels if abs(v) > 0.001]
        self.assertGreater(
            len(non_zero_cmds),
            0,
            'Controller should generate non-zero velocity commands',
        )

        # Verify commands are within expected range (max_speed = 0.5)
        for vel in self.cmd_vels:
            self.assertLessEqual(
                abs(vel), 0.51, 'Velocity exceeds max_speed limit'
            )
