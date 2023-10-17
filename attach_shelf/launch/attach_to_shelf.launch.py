from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the service server node
        Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen',
            parameters=[
                {'final_approach': True}  # Assuming you want to set this parameter here
            ]
        ),
        # Start the obstacle avoidance node
        Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            name='pre_approach_v2',
            output='screen',
            parameters=[
                {'obstacle_distance': 0.3},
                {'degrees': -90.0}
            ]
        )
    ])
