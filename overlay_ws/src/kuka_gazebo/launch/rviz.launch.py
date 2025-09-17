# Author: REZ3LIET

import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy.logging
logger = rclpy.logging.get_logger("kuka_2f140.launch")

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def rewrite_yaml(source_file: str, root_key: str):
    if not root_key:
        return source_file

    with open(source_file, 'r') as file:
        ori_data = yaml.safe_load(file)

    updated_yaml = {root_key: ori_data}
    dst_path = f"/tmp/{time.time()}.yaml"
    with open(dst_path, 'w') as file:
        yaml.dump(updated_yaml, file)
    return dst_path

def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def load_robot(context, *args, **kwargs):
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]) 
    
    return [
        DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='/home/hiwi/hankes/main_ws/src/masterarbeit/kuka_moveit_config/config/moveit.rviz', #TODO Path
        description='Full path to the RVIZ config file to use'),
        start_rviz_cmd
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_robot)
    ])
