from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del launch del otro paquete
    rplidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    rplidar_launch_file = os.path.join(rplidar_launch_dir, 'rplidar.launch.py')
    rviz_config_dir = os.path.join(get_package_share_directory('tesis_launch'), 'rviz', 'laser_scan_config.rviz')


    # Ruta del archivo de configuración de RViz
    rviz_config_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'rviz', 'laser_scan_config.rviz')

    # Incluye el launch file del otro paquete
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file)
    )

    return LaunchDescription([
        rplidar_launch,

        # Lanza RViz2 con el topic LaserScan en /scan y frame 'laser'
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config_dir],
            output="screen"
        ),

        # Nodo de la cámara
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='cama_pub',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
            parameters=[{
                'video_frames': '/camera/image_raw',  # Parámetros adicionales si es necesario
            }]
        ),
        
        # Nodo que procesa la imagen (image subscriber)
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='cama_sus',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),

        # Nodo de publicación de comandos
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='coman_pub',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),
        
        # Nodo de suscripción de comandos
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='coman_sus',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),
    ])
