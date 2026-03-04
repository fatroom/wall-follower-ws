#!/usr/bin/env python3

import time
import unittest

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_testing.actions import ReadyToTest
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description with only controller node."""
    controller = LifecycleNode(
        package='cpp_wall_follower',
        executable='controller',
        name='wall_follower_node',
        namespace='',
    )

    return LaunchDescription([controller, ReadyToTest()])


class TestLifecycle(unittest.TestCase):
    """Integration test for controller lifecycle management."""

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
        self.node = Node('test_lifecycle_node')
        self.cmd_vels = []

        # Create subscription to cmd_vel
        self.cmd_vel_sub = self.node.create_subscription(
            Twist, '/cmd_vel', lambda msg: self.cmd_vels.append(msg.linear.x), 10
        )

        # Create lifecycle service clients
        self.change_state_client = self.node.create_client(
            ChangeState, '/wall_follower_node/change_state'
        )

        self.get_state_client = self.node.create_client(
            GetState, '/wall_follower_node/get_state'
        )

        # Wait for services
        self.assertTrue(
            self.change_state_client.wait_for_service(timeout_sec=5.0),
            'Change state service not available',
        )
        self.assertTrue(
            self.get_state_client.wait_for_service(timeout_sec=5.0),
            'Get state service not available',
        )

    def tearDown(self):
        """Clean up test fixtures."""
        # Reset lifecycle node to unconfigured state for next test
        # First, get current state
        get_state_request = GetState.Request()
        future = self.get_state_client.call_async(get_state_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        if future.done():
            current_state = future.result().current_state.id
            # If in inactive state (2), cleanup to return to unconfigured
            if current_state == 2:  # State: inactive
                self.change_state(Transition.TRANSITION_CLEANUP)

        self.node.destroy_node()

    def change_state(self, transition_id):
        """Send lifecycle transition request."""
        request = ChangeState.Request()
        request.transition.id = transition_id

        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        self.assertTrue(future.done(), 'Change state request timed out')
        response = future.result()
        return response.success

    def spin_for_duration(self, duration_sec):
        """Spin node for specified duration."""
        start_time = time.time()
        while (time.time() - start_time) < duration_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_lifecycle_transitions(self):
        """Test configure -> activate -> deactivate transitions."""
        # Start: unconfigured state

        # Transition to configured
        success = self.change_state(Transition.TRANSITION_CONFIGURE)
        self.assertTrue(success, 'Configure transition failed')

        # Transition to active
        success = self.change_state(Transition.TRANSITION_ACTIVATE)
        self.assertTrue(success, 'Activate transition failed')

        # Wait for messages to start flowing
        self.spin_for_duration(1.0)

        # Verify messages are being published
        initial_msg_count = len(self.cmd_vels)
        self.assertGreater(
            initial_msg_count, 0, 'Should receive cmd_vel messages after activation'
        )

        # Clear message buffer
        self.cmd_vels.clear()

        # Transition to inactive (deactivate)
        success = self.change_state(Transition.TRANSITION_DEACTIVATE)
        self.assertTrue(success, 'Deactivate transition failed')

        # Check that final message is zero velocity
        if len(self.cmd_vels) > 0:
            final_velocity = self.cmd_vels[-1]
            self.assertAlmostEqual(
                final_velocity,
                0.0,
                places=6,
                msg='Final velocity should be zero on deactivation',
            )

        # Wait and verify no more messages
        self.cmd_vels.clear()
        self.spin_for_duration(1.0)

        self.assertEqual(
            len(self.cmd_vels),
            0,
            'Should not receive cmd_vel messages after deactivation',
        )

    def test_deactivation_publishes_zero_velocity(self):
        """Test that deactivation always publishes zero velocity."""
        # Configure and activate
        self.change_state(Transition.TRANSITION_CONFIGURE)
        self.change_state(Transition.TRANSITION_ACTIVATE)

        # Wait for normal operation
        self.spin_for_duration(0.5)
        self.cmd_vels.clear()

        # Deactivate
        self.change_state(Transition.TRANSITION_DEACTIVATE)

        # Spin briefly to receive the final message
        self.spin_for_duration(0.2)

        # Verify at least one message received and it's zero
        self.assertGreater(
            len(self.cmd_vels), 0, 'Should receive at least one message on deactivation'
        )

        final_velocity = self.cmd_vels[-1]
        self.assertAlmostEqual(
            final_velocity,
            0.0,
            places=6,
            msg='Deactivation must publish zero velocity for safety',
        )
