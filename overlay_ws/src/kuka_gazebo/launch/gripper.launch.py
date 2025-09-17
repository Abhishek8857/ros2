from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', '/kuka_gazebo/models/endeffectors/gripper.sdf',
                '-x', '-1.79', '-y', '-0.319', '-z', '1.352', '-R', '1.5707963267948966', '-P', '0.0', '-Y', '3.141592653589793'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=['/model/gripperKUKA/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperKUKA/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperTCS/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty'
                       ]
        )
    ])