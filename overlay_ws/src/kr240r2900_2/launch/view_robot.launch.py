from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "kr240r2900_2"

    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        "urdf",
        f"{package_name}.xacro"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        "rviz",
        "rviz.rviz"
    ])

    robot_description = Command(["xacro ", urdf_path])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    ])
