# REVAMP Demonstrator Simulation

## Getting started

- The kuka_bringup.launch.py launch file was implemented based on the following repo
- kr70_r210 repo: https://github.com/REZ3LIET/KUKA-ROS2.git

## Dependencies

```
sudo apt-get install ros-humble-moveit
sudo apt-get install ros-humble-ros-gz
sudo apt-get install ros-humble-ign-ros2-control
```

## to run the simulation

1. launch the (not so empty) empty_world.sdf 
    ```
    ros2 launch kuka_gazebo empty_world.launch.py
    ```
2. load the kuka robot and the donkey together with moveit. This needs to be executed in order to see the world. (takes some time and has a black/white screen)
    ```
    ros2 launch kuka_gazebo kuka_bringup.launch.py
    ```
3. load the end effectors and the battery pack
    ```
    ros2 launch kuka_gazebo endeffector.launch.py
    ```
4. start the motion sequence
    ```
    ros2 launch kuka_gazebo motion_plan_2.launch.py
    ```
5. (optional and instead of 4.) start rviz to use the moveit motion planning plugin to manually move the kuka/donkey/omnibase
    ```
    ros2 launch kuka_gazebo rviz.launch.py
    ```

## Simulation
The simulation includes the following steps of the remanufacturing process.
- movement of the donkey and omnibase between workspaces
- tool changing between vacuum gripper, module gripper and EIS measurement device
- removal of the battery pack cover
- positioning of the EIS measurement device on different battery modules
- removal of a battery module

The removal of the cover and the battery module and the tool changing are not simulated as physical processes. They are realized through detachable joints.

## Note
The worker is taken from thingiverse and falls under the following license. It can be found
[here](https://www.thingiverse.com/thing:5160039).
![alt text](image.png)

## Open work
- Inclusion of the Zivid camera into the process.
- There is no collision detection for the donkey and the omnibase towards the environment.
- The battery pack has no collision geometry as the mesh is too complex and therefore slows down the simulation to ~2-4% real time. --> simplify collision geometry.
- Workspaces on the floor do not align with the workshop background model.
- The SDF/URDF/XACRO are not 100% the same. The current SDF has the right positions/rotations for all objects.
- when converting XACRO/URDF to SDF (for the kuka), the contol plugin at the bottom of the sdf will need an additional '**s**' in the name of the .yaml file (ros2_controller**s**.yaml). (Not open work but the problem is annoying to find...)
- there are many absolute paths that will need changing on machines other than the hiwi computers.
- There are two screws in the visual of the battery pack that need to be removed.
- the omnibase and the flange are not rotated in the rviz model.
- some names of variables/packages are not optimal...

## Packages

### kuka_gazebo
main package of the simulation. Includes all launch files, the models of the end effectors, the battery pack and the world

### eis_description
provides all meshes, configs, and urdfs of the kuka robot

### kuka_motion
includes the source files used for the action server and client that handle the movement of the KUKA, Omnibase and Donkey. Also includes the motion state generator that holds the motion sequence of the simulation.

### kuka_motion_plan_action
supporting package for kuka_motion. Defines the ROS action and message types needed.

### kuka_moveit_config
Includes all necessary config files for moveit.