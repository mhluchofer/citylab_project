from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_patrol')
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
        ),

        # Lanza RViz2 con la configuración predefinida
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', f'{pkg_share}/rviz/robot_patrol.rviz']
        ),
    ])
