import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    mode = LaunchConfiguration("mode")
    client_ip = LaunchConfiguration("client_ip")
    client_port = LaunchConfiguration("client_port")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    roundtrip_time = LaunchConfiguration("roundtrip_time")
    ns = LaunchConfiguration("namespace")
    controller_config = LaunchConfiguration("controller_config")
    jtc_config = LaunchConfiguration("jtc_config")
    
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"{robot_model.perform(context)}"),
                    "urdf",
                    robot_model.perform(context) + ".xacro",
                ]
            ),
            " ",
            "mode:=",
            mode,
            " ",
            "client_port:=",
            client_port,
            " ",
            "client_ip:=",
            client_ip,
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "x:=",
            x,
            " ",
            "y:=",
            y,
            " ",
            "z:=",
            z,
            " ",
            "roll:=",
            roll,
            " ",
            "pitch:=",
            pitch,
            " ",
            "yaw:=",
            yaw,
            " ",
            "roundtrip_time:=",
            roundtrip_time,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ],
        on_stderr="capture",
    )

    robot_description = {"robot_description": robot_description_content}

    moveit_config = (      
        MoveItConfigsBuilder(robot_name="kr240r2900_2", package_name="kuka_moveit_config")
        .robot_description_semantic(
            get_package_share_directory("kuka_moveit_config")
            + "/config/kr240r2900_2.srdf"
        )     
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, 
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits() 
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description
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
        parameters=[ros2_controllers_path, robot_description],
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
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr240r2900_2"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="quantec"))
    launch_arguments.append(DeclareLaunchArgument("mode", default_value="hardware"))
    launch_arguments.append(DeclareLaunchArgument("use_fake_hardware", default_value="false"))
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0")) # default: 192.168.1.27
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roundtrip_time", default_value="4000"))
    launch_arguments.append(DeclareLaunchArgument("use_sim_time",default_value="false"))
    
    return LaunchDescription(launch_arguments+ [OpaqueFunction(function=launch_setup)])