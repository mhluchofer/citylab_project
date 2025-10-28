from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_patrol')
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='go_to_pose_action',
            name='go_to_pose_action',
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