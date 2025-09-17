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

def load_motion_plan(context, *args, **kwargs):

    pose_vector_publisher = Node(
        package="kuka_motion",
        executable="motion_state_generator",
        name="motion_state_generator",
        output="both"
    )

    kuka_motion_server = Node(
        package="kuka_motion",
        executable="kuka_motion",
        name="kuka_motion_server",
        output="both"
    )

    kuka_motion_client = Node(
        package="kuka_motion",
        executable="kuka_motion_client_TCS",
        name="kuka_motion_client",
        output="both"
    )
    

    return [
        pose_vector_publisher,
        kuka_motion_server,
        kuka_motion_client
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_motion_plan)
    ])
