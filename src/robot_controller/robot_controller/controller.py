import rclpy
from robot_controller.scripts.rosmaster import Rosmaster
from geometry_msgs.msg import Twist
from rclpy.node import Node

import atexit

TWIST_ZERO = Twist()

class Controller(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        self.bot = Rosmaster(car_type=2)
        self.bot.create_receive_threading()
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.vel_publisher = self.create_publisher(Twist, '/vel_raw', 10)

        atexit.register(self.stop)

    def stop(self):
        self.handle_cmd_vel(TWIST_ZERO)
    
    def handle_cmd_vel(self, msg: Twist):
        self.bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
        self.vel_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()
