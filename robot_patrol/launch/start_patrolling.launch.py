from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',         # nombre de tu paquete
            executable='patrol_node',       # nombre del ejecutable (de CMakeLists)
            name='robot_patrol',            # nombre del nodo
            output='screen',                # muestra logs en la terminal
            emulate_tty=True,               # mantiene colores en la salida (opcional)
        )
    ])