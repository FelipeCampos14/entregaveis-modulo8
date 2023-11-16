import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.create_timer(1.0, self.timer_callback)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel')
        self.create_publisher

    def timer_callback(self):   
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)   
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()    

if __name__ == '__main__':
    main()