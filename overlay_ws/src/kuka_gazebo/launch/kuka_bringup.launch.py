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
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    robot_name = LaunchConfiguration('robot_name', default='kuka_arm')
    namespace = LaunchConfiguration('namespace', default='')
    gripper_name = LaunchConfiguration('gripper_name', default='')
    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')
    orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_name_val = robot_name.perform(context)
    namespace_val = namespace.perform(context)
    gripper_name_val = gripper_name.perform(context)
    logger.info(f"Loading Kuka Robot [{gripper_name_val}] with name: {robot_name_val}, namespace: {namespace_val}")
    
    # if gripper_name_val == "robotiq_2f_85":
    #     controller_yaml = "config/kuka_2f85_controllers.yaml"
    #     moveit_pkg = "kuka_2f85_moveit"
    # elif gripper_name_val == "robotiq_2f_140":
    #     controller_yaml = "config/kuka_2f140_controllers.yaml"
    #     moveit_pkg = "kuka_2f140_moveit"
    # else:
    controller_yaml = "config/kuka_controllers.yaml"
    moveit_pkg = "kuka_moveit_config"

    # joint_state_publisher_gui = Node(
    #     package= 'joint_state_publisher_gui',
    #     executable= 'joint_state_publisher_gui',
    #     name= 'joint_state_publisher_gui'
    # )

    # Updating controller parameters
    sim_dir = get_package_share_directory('kuka_gazebo')
    pkg_dir = get_package_share_directory('eis_description')
    controller_file = rewrite_yaml(
        source_file=os.path.join(sim_dir, controller_yaml),
        root_key=namespace_val
    )

    # Loading Robot Model
    robot_xacro = Command([
        'xacro ', os.path.join(pkg_dir, 'urdf/kr240r2900_2.urdf.xacro'),
        ' robot_name:=', robot_name,
        ' namespace:=', namespace,
        ' gripper_name:=', gripper_name,
        ' controller_file:=', controller_file,
        ])
    robot_description = {"robot_description": robot_xacro}

    # Publish TF
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        remappings=remappings,
        output="both",
        parameters=[robot_description]
    )

    # Launch RViz
    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file]) 
    
    # Spawn Robot in Gazebo
    # spawn_robot_node = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-topic", f"{namespace_val}/robot_description",
    #         "-name", robot_name,
    #         "-robot_namespace", namespace,
    #         "-x", position_x,
    #         "-y", position_y,
    #         "-Y", orientation_yaw,
    #     ],
    #     output="both"
    # )

    # Load Controllers
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )

    kuka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kuka_arm_controller', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )

    plug_1_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'plug_1_controller', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )

    plug_2_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'plug_2_controller', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )
    
    omnibase_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omnibase_controller', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )

    agv_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'agv_controller', '-c', f"{namespace_val}/controller_manager"],
        output="screen"
    )

    robotiq_controller = None
    if gripper_name_val == "robotiq_2f_85":
        robotiq_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robotiq_2f85_controller', '-c', f"{namespace_val}/controller_manager"],
            output="screen"
        )
    elif gripper_name_val == "robotiq_2f_140":
        robotiq_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robotiq_2f140_controller', '-c', f"{namespace_val}/controller_manager"],
            output="screen"
        )
    else:
        robotiq_controller = ExecuteProcess(
            cmd=['echo', '"No grippers loaded"'],
            output="screen"
        )

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[
                joint_state_controller,
                kuka_controller,
                # plug_1_controller,
                # plug_2_controller,
                omnibase_controller,
                agv_controller
            ]
        )
    )

    controller_config = os.path.join(
        get_package_share_directory('kuka_moveit_config'),
        'config',
        'ros2_controllers.yaml'
    )

    Node(
        package='ros_gz_control',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )


    moveit_config = (
        MoveItConfigsBuilder("kuka_moveit_config", package_name=moveit_pkg)
        .robot_description(
            file_path="config/kr240r2900_2.urdf.xacro",
            mappings={
                "robot_name": robot_name_val,
                "namespace": namespace_val,
                "gripper_name": gripper_name_val,
                "controller_file": controller_file,
            }
        )
        .planning_pipelines("pilz_industrial_motion_planner")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .robot_description_semantic(file_path="config/kr240r2900_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    planning_scene_parameters={
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }

    ompl_planning_yaml = load_yaml(
        get_package_share_directory(moveit_pkg), "config/ompl_planning.yaml"
    )

    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    pilz_pipeline = {
            'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner', 
            # 'request_adapters': 'default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints', 
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            'default_planner_config': 'PTP', 
            'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'
        }
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace_val,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_parameters,
            ompl_planning_pipeline_config,
            pilz_pipeline,
            {"use_sim_time": use_sim_time}
        ]
    )

    load_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=agv_controller,
            on_exit=[move_group_node]
        )
    )

    # *** PLANNING CONTEXT *** #
    # Robot description SRDF
    # robot_description_semantic_config = load_file(moveit_pkg, "config/kr240r2900_2.srdf")
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # # Kinematics YAML file
    # kinematics_yaml = load_yaml(get_package_share_directory(moveit_pkg), "config/kinematics.yaml")

    # MoveInterface = Node(
    #     package="control_scripts",
    #     executable="move_robot",
    #     output="screen",
    #     parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ENV_PARAM": "gazebo"}],
    # )

    return [
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value=use_sim_time,
            description="Use simulator time",
            choices=["True", "False"]
        ),
        DeclareLaunchArgument(
            name="robot_name",
            default_value=robot_name,
            description="Name of the robot"
        ),
        DeclareLaunchArgument(
            name="namespace",
            default_value=namespace,
            description="Namespace for the robot"
        ),
        DeclareLaunchArgument(
            name="gripper_name",
            default_value=gripper_name,
            description="Name of gripper to use",
            choices=["", "robotiq_2f_85", "robotiq_2f_140"]
        ),
        DeclareLaunchArgument(
            name="position_x",
            default_value=position_x,
            description="X position to spawn the robot"
        ),
        DeclareLaunchArgument(
            name="position_y",
            default_value=position_y,
            description="Y position to spawn the robot"
        ),
        DeclareLaunchArgument(
            name="orientation_yaw",
            default_value=orientation_yaw,
            description="Yaw angle to spawn robot"
        ),
        DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='/kuka_moveit_config/config/moveit.rviz', #TODO Path
        description='Full path to the RVIZ config file to use'),
        # start_rviz_cmd,
        #joint_state_publisher_gui,
        robot_state_publisher,
        # spawn_robot_node,
        # load_controllers,
        joint_state_controller,
        kuka_controller,
        # plug_1_controller,
        # plug_2_controller,
        omnibase_controller,
        agv_controller,
        load_move_group
        # MoveInterface
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_robot)
    ])
