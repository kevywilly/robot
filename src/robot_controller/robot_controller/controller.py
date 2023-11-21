import rclpy
from robot_controller.scripts.rosmaster import Rosmaster
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy import qos
import atexit
import numpy as np # Scientific computing library for Python
 
class Quarternion:
    def __init__(self, x, y, z):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        self.x = np.sin(x/2) * np.cos(y/2) * np.cos(z/2) - np.cos(x/2) * np.sin(y/2) * np.sin(z/2)
        self.y = np.cos(x/2) * np.sin(y/2) * np.cos(z/2) + np.sin(x/2) * np.cos(y/2) * np.sin(z/2)
        self.z = np.cos(x/2) * np.cos(y/2) * np.sin(z/2) - np.sin(x/2) * np.sin(y/2) * np.cos(z/2)
        self.w = np.cos(x/2) * np.cos(y/2) * np.cos(z/2) + np.sin(x/2) * np.sin(y/2) * np.sin(z/2) 
  

TWIST_ZERO = Twist()
VELOCITY_RATIO = 0.66

class Controller(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        self.bot = Rosmaster(car_type=2)
        self.bot.create_receive_threading()
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, qos.qos_profile_services_default)
        self.vel_publisher = self.create_publisher(Twist, '/vel_raw', 10)
        self.odom_timer = self.create_timer(0.2, self.odom_trigger);
        
        atexit.register(self.stop)

    def stop(self):
        self.handle_cmd_vel(TWIST_ZERO)

    def handle_cmd_vel(self, msg: Twist):
        self.bot.set_car_motion(msg.linear.x * VELOCITY_RATIO, msg.linear.y * VELOCITY_RATIO ,msg.angular.z * VELOCITY_RATIO)

    def odom_trigger(self, *args):
        msg = Twist()
        x,y,z = self.bot.get_motion_data()
        msg.linear.x = x/VELOCITY_RATIO
        msg.linear.y = y/VELOCITY_RATIO
        msg.angular.z = z/VELOCITY_RATIO

        self.vel_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()
