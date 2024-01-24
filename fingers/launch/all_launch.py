import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 

def generate_launch_description():
    imus = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('imu'), 'launch'),
         '/bringup_imus_launch.py'])
      )
    tofs = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('tof'), 'launch'),
         '/tof_bringup_launch.py'])
      )
    motors = Node(
            package='fingers',
            executable='motor_control',
            name='motor_control',
            output="screen",
            emulate_tty=True)
    return LaunchDescription([imus, tofs, motors])