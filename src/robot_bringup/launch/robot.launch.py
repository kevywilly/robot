import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import launch_ros
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

lidar = Node(
         package='sllidar_ros2',
         executable='sllidar_node',
         name='sllidar_node',
         parameters=[
               {'channel_type': 'serial'},
               {'serial_port': '/dev/rplidar'},
               {'serial_baudrate': 115200},
               {'frame_id': 'laser'},
               {'inverted','false'}
         ],
   )
robot_app = Node(
   package='robot',
   executable='app',
   name='robot_app'
)

robot_controller = Node(
   package='robot_controller',
   executable='controller',
   name='robot_controller'
)

robot_odometry = Node(
   package='robot_odometry',
   executable='odometry',
   name='robot_odometry'
)

argus_camera_node = Node(
      package='robot_argus_camera',
      executable='camera',
      name='camera',
      output='screen',
      parameters = [
         {'num_cameras': 2}
      ]
   )

def generate_launch_description():
   
   return LaunchDescription([                                  
      argus_camera_node,
      robot_app,
      robot_controller,
      robot_odometry,
      lidar
   ])