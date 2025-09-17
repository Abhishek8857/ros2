from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', '/kuka_gazebo/models/endeffectors/eis_endeffector.sdf',
                '-x', '-1.79', '-y', '0.36', '-z', '1.108', '-R', '-1.5707963267948966', '-P', '0.0', '-Y', '-1.5707963267948966'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=['/model/eisKUKA/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisKUKA/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisTCS/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty'
                       ]
        )
    ])