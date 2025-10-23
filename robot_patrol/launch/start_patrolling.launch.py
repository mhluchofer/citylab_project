from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():

    pkg_share = get_package_share_directory('robot_patrol')
    return LaunchDescription([
        Node(
            package='robot_patrol',         # nombre de tu paquete
            executable='patrol_node',       # nombre del ejecutable (de CMakeLists)
            name='robot_patrol',            # nombre del nodo
            output='screen',                # muestra logs en la terminal
            emulate_tty=True,               # mantiene colores en la salida (opcional)
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