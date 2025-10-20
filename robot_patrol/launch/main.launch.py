from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the direction service server first
        Node(
            package='robot_patrol',
            executable='direction_service',
            name='direction_service',
            output='screen'
        ),
        # Start the patrol node that uses the service
        Node(
            package='robot_patrol',
            executable='patrol_with_service',
            name='patrol_with_service',
            output='screen'
        )
    ])
