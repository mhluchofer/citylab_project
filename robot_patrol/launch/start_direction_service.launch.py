from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',      # Nombre de tu paquete
            executable='direction_service',  # Nombre del ejecutable (de tu CMakeLists.txt)
            name='direction_service',    # Nombre del nodo
            output='screen'              # Mostrar logs en la terminal
        )
    ])
