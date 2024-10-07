import subprocess
import rclpy
from rclpy.node import Node

class RvizLauncherNode(Node):
    def __init__(self):
        super().__init__('rviz_launcher_node')
        self.get_logger().info('Launching RViz2 with LaserScan topic /scan and frame laser...')
        
        # Ejecutar rviz2 en otra terminal
        subprocess.Popen(['gnome-terminal', '--', 'rviz2', '-d', '/home/cliford/tesis_ros/rviz/laser_scan_config.rviz'])

def main(args=None):
    rclpy.init(args=args)
    node = RvizLauncherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
