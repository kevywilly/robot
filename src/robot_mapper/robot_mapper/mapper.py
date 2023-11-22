import rclpy
from robot_controller.scripts.rosmaster import Rosmaster
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy import qos
import numpy as np
import atexit
import numpy as np # Scientific computing library for Python
import math
ROBOT_WIDTH = 0.2748
ROBOT_LENGTH = 0.208
  
SAFE_RADIUS = ROBOT_WIDTH
SAFE_RANGE = abs(math.acos(.5*ROBOT_WIDTH/(ROBOT_WIDTH*2))-math.acos(-.5*ROBOT_WIDTH/(ROBOT_WIDTH*2)))
MAX_RADIANS = 6.283185307
THRESHOLD = ROBOT_WIDTH + ROBOT_WIDTH
#cos A = b/c, cos B = a/c.
# a = acos(b/c)
DEGREES = 57.2958
TOF_MAX = 2500

IDX_RADIANS = 0
IDX_DEGREES = 1 

class Mapper(Node):
    def __init__(self):
        super().__init__("mapper", parameter_overrides=[])
        
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos.qos_profile_services_default)
        self.create_subscription(Odometry, "/odom_raw", self.odom_handler, 50)
        self.create_subscription(Bool, "/autodrive", self.autodrive_handler, qos.qos_profile_services_default)
        self.create_subscription(LaserScan, "/scan", self.scan_handler, 10)
        self.create_subscription(Range, "/tof_raw", self.tof_handler, 50)
        self.create_subscription(TransformStamped, "tf", self.tf_handler, 10)
        self.set_autodrive(False)
        self.prev_heading = None
        self.tof_meters: float = 12.5

    def autodrive_handler(self, msg: Bool):
        self.set_autodrive(msg.data)
    
    def set_autodrive(self, status: bool):
        self.target = 0
        self.heading = None
        self.tof_meters: float = 12.5
        self.best_options = (None, None, None)
        self.autodrive = status
        self.cmd_vel_publisher.publish(Twist())

    def tf_handler(self, tf: TransformStamped):
        pass

    def tof_handler(self, range: Range):
        self.tof_meters = range.range / 1000.0 + ROBOT_LENGTH / 2.0

    def odom_handler(self, odom: Odometry):
        pass

    def print_readings(self, readings):
        for reading in readings:
                self.get_logger().info(f"{reading[0]}, {reading[1], reading[2]}")

    def calc_heading(self, opt):
        result = None
        if not opt[0] and not opt[1] and not opt[2]:
            return None, 1
        
        if opt[1] is not None:
            result =  opt[1]
        elif not opt[0] is not None:
            result = opt[2]
        elif not opt[2] is not None:
            result = opt[0]
        else:
            result = opt[0] if abs(self.target-opt[0]) < abs(self.target-opt[2]) else opt[2]

        return result, abs(self.target - result)/6.14
        
    
    def calc_best_options(self, target: int, options: set):
        less = None
        greater = None
        match = None
        for option in options:
            if option == target:
                match = option
            else:
                diff = abs(target-option)
                if option < target:
                    if not less or diff < abs(target-less):
                        less = option
                else:
                    if not greater or diff < abs(target-greater):
                        greater = option

        return (less,match,greater)

    def scan_handler(self, scan: LaserScan):
        tof_min_angle = -12.5 * 0.017
        tof_max_angle = 12.5 * 0.017
        r = scan.angle_min
        clear_count = int(SAFE_RANGE/scan.angle_increment)
        readings = []
        safe = []
        options = set()
        for range in scan.ranges:
            value = scan.range_max if math.isinf(range) else range

            #if(r >= tof_min_angle and r <= tof_max_angle):
                # value = min(self.tof_meters, value)
            #    pass

            # reference frame is reversed - robot thinks in counter clockwise
            measure = (r, r*DEGREES, value)

            readings.append(measure)
            r=r+scan.angle_increment
            
            if value >= THRESHOLD:
                safe.append(measure[IDX_DEGREES])
            else:
                safe = []
            if len(safe) >= clear_count:
                option = sum(safe)/len(safe)
                options.add(int(option))
                safe.pop(0)
            
        best_options = self.calc_best_options(self.target, options)
        degrees, err = self.calc_heading(best_options)

        self.get_logger().info(f"options: {best_options}, tof: {self.tof_meters}, deg: {degrees}")

        twist = Twist()

        if degrees is not None:
            if degrees > 0:
                twist.angular.z = 1.5
            elif degrees < 0:
                twist.angular.z = -1.5
            else:
                twist.linear.x = 0.35

        self.cmd_vel_publisher.publish(twist)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    rclpy.spin(node)
    rclpy.shutdown()
