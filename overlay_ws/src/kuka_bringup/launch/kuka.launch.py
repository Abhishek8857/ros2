import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # use_sim_time = LaunchConfiguration("use_sim_time")

    
    moveit_config = (
        MoveItConfigsBuilder("kr240r2900_2", package_name="kuka_moveit_config")
        .robot_description()        
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, 
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits() 
        .to_moveit_configs()
    )
    
    # moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context)=="true"})
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ]
    )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"]
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        respawn=True,
        output="both",
        parameters=[
            moveit_config.robot_description
        ]
    )
    
    ros2_controllers_path = os.path.join(get_package_share_directory("kuka_moveit_config"), "config", "ros2_controllers.yaml")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both"
    )
    
    robot_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"]
    )
    
    rviz_config = os.path.join(get_package_share_directory("kuka_moveit_config"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )
    
    delay_rviz_after_joint_state_broadcast_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )
    
    nodes_to_start = [
     ros2_control_node,
     robot_state_publisher,
     joint_state_broadcaster_spawner,
     delay_rviz_after_joint_state_broadcast_spawner,
     robot_trajectory_controller_spawner,
     move_group_node,
     static_tf  
    ]
    
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock"
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])