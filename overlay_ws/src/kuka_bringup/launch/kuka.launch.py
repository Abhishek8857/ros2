import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    # Moveit Config
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
    

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("kr240_r2900_2"),
            "urdf",
            "kr240_r2900_2.urdf.xacro"
        ]),
    ])

    moveit_config.robot_description = {"robot_description": robot_description} 
    moveit_config.moveit_cpp.update({"use_sim_time": False})
    

    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"]
    )

    rviz_config_file = PathJoinSubstitution([
            FindPackageShare("kuka_moveit_config"),
            "config",
            "moveit.rviz"
        ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription([
        move_group_node,
        static_tf,
        rviz_node
    ])