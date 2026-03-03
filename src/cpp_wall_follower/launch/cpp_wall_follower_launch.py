from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    controller = LifecycleNode(
        package='cpp_wall_follower',
        executable='controller',
        name='velocity_controller_node',
        namespace=''
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
            package='cpp_wall_follower',
            executable='sensor',
            name='distance_sensor_node'
        ),
        Node(
            package='cpp_wall_follower',
            executable='filter',
            name='distance_filter_node',
            parameters=[{'alpha': 0.1}]
        ),
        controller,
        activate_on_configure,
        configure_event,
    ])
