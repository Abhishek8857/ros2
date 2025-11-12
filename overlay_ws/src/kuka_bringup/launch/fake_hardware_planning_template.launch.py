# Copyright 2022 Aron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration("dof")

    rviz_config_file = PathJoinSubstitution([
            FindPackageShare("kuka_moveit_config"),
            "config",
            "moveit.rviz"
        ])
    
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kr240_r2900_2"),
                    "urdf",
                    # "kr240_r2900_2" + ".urdf.xacro",
                    "omnimove_with_kr240_r2900_2" + ".urdf.xacro"
                ]
            ),
            " ",
            "mode:=mock",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Create MoveIt config with our full robot description
    moveit_config = (      
        MoveItConfigsBuilder(robot_name="kr240_r2900_2", package_name="kuka_moveit_config")
        .robot_description_semantic(
            get_package_share_directory("kuka_moveit_config")
            + "/config/kr240_r2900_2.srdf"
        )     
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, 
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits() 
        .to_moveit_configs()
    )
    
    # Merge robot description into moveit_config dictionary
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update(robot_description)
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict]
    )
    
    controller_config = (
        get_package_share_directory("kuka_resources")
        + f"/config/fake_hardware_config_{dof.perform(context)}_axis.yaml"
    )

    controller_manager_node = "/controller_manager"

    # Merge controller config with robot description
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
    )

    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_dict],
    )
    

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers
    # def controller_spawner(controller_with_config):
    #     arg_list = [
    #         controller_with_config[0],
    #         "-c",
    #         controller_manager_node,
    #         "-p",
    #         controller_with_config[1],
    #     ]
    #     return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    def controller_spawner(controller_with_config):
        args = [controller_with_config[0], "-c", controller_manager_node]
        if controller_with_config[1]:  # only add -p if file exists
            args += ["-p", controller_with_config[1]]
        return Node(package="controller_manager", executable="spawner", arguments=args)

    controller_names_and_config = [
        ("joint_state_broadcaster", []),
        ("joint_trajectory_controller", controller_config),
    ]

    controller_spawners = [
        controller_spawner(controllers) for controllers in controller_names_and_config
    ]

    to_start = [control_node, robot_state_publisher, rviz_node, move_group_node] + controller_spawners

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("dof", default_value="6"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])


