import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET


def generate_launch_description():
    spawn_models = []
    module_bridges = []
    
    gazebo_pkg_path = get_package_share_directory('kuka_gazebo')
    models_path = os.path.join(gazebo_pkg_path, 'models', 'temp')
    sdf_file = os.path.join(gazebo_pkg_path, 'models', 'battery_pack_layer', 'single_battery_module.sdf')
    os.makedirs(models_path, exist_ok=True)

    tree = ET.parse(sdf_file)
    root = tree.getroot()

    module_poses = [
        ['6.0657', '4.1512', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '4.3155', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '4.4795', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '4.6655', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '4.8295', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '4.9935', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '5.1575', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '5.3215', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '5.5075', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '5.6715', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['6.0657', '5.8355', '0.5351', '-1.5707963267948966', '0', '1.5707963267948966'],
        ['5.7851', '4.3605', '0.5351', '-1.5707963267948966', '0', '3.141592653589793'],
        ['5.7851', '4.7845', '0.5351', '-1.5707963267948966', '0', '3.141592653589793'],
        ['5.7851', '5.2025', '0.5351', '-1.5707963267948966', '0', '3.141592653589793'],
        ['5.7851', '5.6265', '0.5351', '-1.5707963267948966', '0', '3.141592653589793']
    ]

    for i in range(15):
        root[0].attrib["name"]=f"module_{i}"
        root[0][1].attrib["name"]=f"module_{i}"
        root[0][3][0].text = f"module_{i}"
        root[0][3][3].text = f"/model/module_{i}Pack/detach"
        root[0][3][4].text = f"/model/module_{i}Pack/attach"
        root[0][4][0].text = f"module_{i}"
        root[0][4][3].text = f"/model/module_{i}Gripper/detach"
        root[0][4][4].text = f"/model/module_{i}Gripper/attach"
        tree.write(os.path.join(models_path, f"model_{i}.sdf"))
        module_bridges.append(f'/model/module_{i}Pack/detach@std_msgs/msg/Empty]ignition.msgs.Empty')
        module_bridges.append(f'/model/module_{i}Pack/attach@std_msgs/msg/Empty]ignition.msgs.Empty')
        module_bridges.append(f'/model/module_{i}Gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty')
        module_bridges.append(f'/model/module_{i}Gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty')
        spawn_models.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', 'empty',
                    '-file', os.path.join(models_path, f"model_{i}.sdf"),
                    '-x', module_poses[i][0], '-y', module_poses[i][1], '-z', module_poses[i][2], '-R', module_poses[i][3], '-P', module_poses[i][4], '-Y', module_poses[i][5]
                ],
                output='screen'
            )
        )
        

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', os.path.join(gazebo_pkg_path, 'models', 'endeffectors', 'vakuumgreifer.sdf'),
                '-x', '-2.194', '-y', '0.02', '-z', '1.526', '-R', '0.0', '-P', '0.0', '-Y', '1.5707963267948966'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', os.path.join(gazebo_pkg_path, 'models', 'endeffectors','gripper.sdf'),
                '-x', '-1.79', '-y', '0.021', '-z', '1.352', '-R', '1.5707963267948966', '-P', '0.0', '-Y', '1.5707963267948966'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', os.path.join(gazebo_pkg_path, 'models', 'endeffectors','eis_endeffector.sdf'),
                '-x', '-1.79', '-y', '0.43', '-z', '1.108', '-R', '-1.5707963267948966', '-P', '0.0', '-Y', '-1.5707963267948966'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', os.path.join(gazebo_pkg_path, 'models', 'battery_pack_layer', 'battery_pack.sdf'),
                '-x', '6.0', '-y', '5.0', '-z', '0.39', '-R', '-1.5707963267948966', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),
        *spawn_models,
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file', os.path.join(gazebo_pkg_path, 'models', 'battery_pack_layer', 'deckel.sdf'),
                '-x', '6.0', '-y', '5.0', '-z', '0.39', '-R', '-1.5707963267948966', '-P', '0.0', '-Y', '0.0'
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
                       '/model/vacuumTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperKUKA/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperKUKA/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperTCS/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/gripperTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisKUKA/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisKUKA/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisTCS/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/eisTCS/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       *module_bridges,
                       '/model/deckel/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/deckel/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/deckelVakuum/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                       '/model/deckelVakuum/attach@std_msgs/msg/Empty]ignition.msgs.Empty'
                       ]
        )
    ])