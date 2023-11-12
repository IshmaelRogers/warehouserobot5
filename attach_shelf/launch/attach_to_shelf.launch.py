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
    
    ])
