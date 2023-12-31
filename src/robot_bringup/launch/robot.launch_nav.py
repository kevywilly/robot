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

pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')


default_model_path = os.path.join(pkg_share, 'src/urdf/robot.urdf')
default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

slam2 = IncludeLaunchDescription(
   PythonLaunchDescriptionSource(
   os.path.join(get_package_share_directory("slam_toolbox"),"launch","online_async_launch.py")
   )
)

nav2 = IncludeLaunchDescription(
   PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory("nav2_bringup"),"launch", "navigation_launch.py")
   ),
   launch_arguments={
            'use_sim_time': 'false'
      }.items()
)


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

rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      parameters=[
         {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
         {'use_tf_static': 'false'}
         ]
      
   )

robot_mapper = Node(
   package="robot_mapper",
   executable="mapper",
   name="mapper"
)


cam1_node = ComposableNode(
   name='argus_mono_0',
   package='isaac_ros_argus_camera',
   plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
   namespace='cam0',
   parameters=[
               {'camera_id': 0},
               {'module_id': 0},
               {'mode': 4},
               {'camera_info_url': "file:///workspaces/camera/camera.ini"}
               ],
)

cam2_node = ComposableNode(
   name='argus_mono_1',
   package='isaac_ros_argus_camera',
   plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
   namespace='cam1',
   parameters=[
               {'camera_id': 1},
               {'module_id': 1},
               {'mode': 4},
               {'camera_info_url': "file:///workspaces/camera/camera.ini"}
               ],
)

argus_contaner = ComposableNodeContainer(
      name='argus_mono_container',
      package='rclcpp_components',
      executable='component_container_mt',
      composable_node_descriptions=[cam1_node, cam2_node],
      namespace='',
      output='screen',
      arguments=['--ros-args', '--log-level', 'info'],
   )

slam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("slam_toolbox"),"launch","online_async_launch.py")
        )
    )
slam = LaunchConfiguration('slam')
use_sim_time = LaunchConfiguration('use_sim_time')

def generate_launch_description():
   

   return LaunchDescription([
      DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
      DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),    

      DeclareLaunchArgument(name='robot_publisher_config', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),                                    
      argus_camera_node,
      robot_state_publisher_node,
      robot_app,
      robot_controller,
      robot_odometry,
      lidar,
      nav2,
      slam2,
      rviz_node,
      robot_mapper
   ])