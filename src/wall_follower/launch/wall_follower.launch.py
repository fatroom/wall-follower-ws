import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('wall_follower'),
        'config',
        'wall_follower_params.yaml'
    )

    controller = LifecycleNode(
        package='wall_follower',
        executable='controller',
        name='wall_follower_node',
        namespace='',
        parameters=[config_file]
    )

    # Emit configure event
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == controller,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # When configured, emit activate event
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == controller,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='sensor',
            name='distance_sensor_node'
        ),
        Node(
            package='wall_follower',
            executable='filter',
            name='distance_filter_node',
            parameters=[config_file]
        ),
        controller,
        activate_on_configure,
        configure_event,
    ])
