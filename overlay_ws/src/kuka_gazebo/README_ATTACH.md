# How to attach/detach models to the robot

- spawn world with `ros2 launch kuka_gazebo empty_world.launch.py` + `ros2 launch kuka_gazebo kuka_bringup.launch.py`
- spawn a model with `ros2 run ros_gz_sim create -world 'empty' -file '/home/hiwi/hankes/main_ws/src/masterarbeit/kuka_gazebo/models/eis_ee/eis_ee.sdf'`
    - `eis_ee.sdf` contains the `DetachableJoint` plugin that specifies the detachable link and the detach and attach topic
- attach and detach the joint by using `ign topic -t "<attach/detach topic>" -m ignition.msgs.Empty -p "unused: true"`
- run rviz with `ros2 launch kuka_gazebo rviz.launch.py`

- spawn gripper in TCS `ros2 run ros_gz_sim create -world 'empty' -file '/home/hiwi/hankes/main_ws/src/masterarbeit/kuka_gazebo/models/endeffectors/gripper.sdf' -x -1.79 -y 0.358 -z 1.35 -R 1.5708`