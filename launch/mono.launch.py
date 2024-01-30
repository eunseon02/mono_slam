import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "rviz2", "rviz2", "-d", "/home/eunseon/ros2_ws/src/mono_slam/rviz/mono.rviz"], output="screen"
        ),

        Node(
            package='mono_slam',
            executable='mono_slam',
            name='mono_slam',
            # parameters=[param_dir],
            output='screen'),


   ])