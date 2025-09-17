from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', '/home/hiwi/hankes/main_ws/src/masterarbeit/kuka_gazebo/models/endeffectors/vakuumgreifer.sdf',
                '-x', '-2.194', '-y', '0.02', '-z', '1.526', '-R', '0.0', '-P', '0.0', '-Y', '1.5707963267948966'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=['/model/vacuumKUKA/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/vacuumKUKA/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/vacuumTCS/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/vacuumTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty'
                       ]
        )
    ])