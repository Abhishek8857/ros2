import os
import yaml
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy.logging
logger = rclpy.logging.get_logger("empty_world.launch")

def generate_launch_description():
    ign_gz = LaunchConfiguration('ign_gz', default='True')

    SetEnvironmentVariable(name='IGN_LOG_LEVEL', value='4'),

    # Loading Gazebo
    world = os.path.join(get_package_share_directory("kuka_gazebo"), "world/empty_world.sdf")
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
        condition=IfCondition(ign_gz)
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name="ign_gz",
            default_value='True',
            description="Use gazebo simulation",
            choices=["True", "False"]
        ),
        ign_gazebo_node,
    ])
