# KUKA Simulation

## Requirements
- Ubuntu 22.04 (tested)
- ROS 2 Humble
- Docker ≥ 20.10
- Recommended: ≥ 16 GB RAM, ≥ 4 CPU cores
- NVIDIA GPU (recommended for better Gazebo performance)


## Getting Started

### 1. Clone the Repo

``` sh 
git clone --recurse-submodules https://git-ce.rwth-aachen.de/wzl-mq-ms/forschung-lehre/revamp/simulation-2.0.git
```

If you’ve already cloned the repository without submodules, you can initialize and update the submodules like this:

```sh
cd simulation-2.0
git submodule update --init --recursive
```

### 2. Build docker image


Build the Docker image using the provided Dockerfile. This command must be run from the root of the repository where the Dockerfile is located. The Docker image name is defined in **`container_name.cfg`**. Edit this file if you want to change the image name before building:

**NOTE: If you have system with low ram storage, please run the docker build with `-low` arg as shown below, else your system will crash during build as colcon tries to build multiple packages at once**

```sh
bash docker_build.sh -low
```

If you have enough memory on your system, run the following docker build
```
bash docker_build.sh
```

This will create a Docker image with the default name from the file, which includes the ROS 2 environment and the Moveit2 and Moveit Task Constructor package located in `colcon_ws`. The simulation packages are located in the `overlay_ws/`

**The build process may take some time, especially on systems with lower RAM storage. Expect around 30 minutes of build time. Grab a coffee ☕**


### 3. Run the Docker Container

Once the Docker image is built, you can run the container interactively using:

```sh
cd docker_run/
bash docker_run.sh
```

### 4. Launch the Simulation

**Step 1: Launch empty world**  

```sh
ros2 launch kuka_gazebo empty_world.launch.py
```
Gazebo will launch with a black screen and output an error in the Terminal:
`[ign gazebo-1] [ERROR] [timestamp] [gz_ros2_control]: robot_state_publisher service not available, waiting again...`
This is expected behavior, the simulation will proceed once the workspace is sourced in the next step.

**Step 2: Open another window in terminal and source the workspace**
```sh
bash docker_exec.sh
source install/setup.bash
```

**Step 3: Launch the KUKA Robot and the Donkey with Moveit**
```sh
ros2 launch kuka_gazebo kuka_bringup.launch.py
```
Now Gazebo will load the world with the robots and the environments, may take some time depending on your system capabilities

**Step 4: Repeat step `2` and execute the following command to load the end effectors and the battery pack**
```sh
ros2 launch kuka_gazebo endeffector.launch.py
```
**Step 5: Repeat step `2` and execute the following command to start the motion sequence**  
```sh 
ros2 launch kuka_gazebo motion_plan_2.launch.py
```

## Simulation
The simulation includes the following steps of the remanufacturing process.
- Movement of the donkey and omnibase between workspaces
- Tool changing between vacuum gripper, module gripper and EIS measurement device
- Removal of the battery pack cover
- Positioning of the EIS measurement device on different battery modules
- Removal of a battery module

The removal of the cover and the battery module and the tool changing are not simulated as physical processes. They are realized through detachable joints.


## Note
The worker is taken from thingiverse and falls under the following license. It can be found
[here](https://www.thingiverse.com/thing:5160039).
![alt text](image.png)

## Known Issues / TODO
- Inclusion of the Zivid camera into the process.
- There is no collision detection for the donkey and the omnibase towards the environment.
- The battery pack has no collision geometry as the mesh is too complex and therefore slows down the simulation to ~2-4% real time. --> simplify collision geometry.
- Workspaces on the floor do not align with the workshop background model.
- The SDF/URDF/XACRO are not 100% the same. The current SDF has the right positions/rotations for all objects.
- When converting XACRO/URDF to SDF (for the kuka), the control plugin at the bottom of the sdf will need an additional '**s**' in the name of the .yaml file (ros2_controller**s**.yaml). (Not open work but the problem is annoying to find...)
- There are many absolute paths that will need changing on machines other than the hiwi computers.
- There are two screws in the visual of the battery pack that need to be removed.
- The omnibase and the flange are not rotated in the rviz model.
- Some names of variables/packages are not optimal...

## Packages

### kuka_gazebo
Main package of the simulation. Includes all launch files, the models of the end effectors, the battery pack and the world

### eis_description
Provides all meshes, configs, and urdfs of the kuka robot

### kuka_motion
Includes the source files used for the action server and client that handle the movement of the KUKA, Omnibase and Donkey. Also includes the motion state generator that holds the motion sequence of the simulation.

### kuka_motion_plan_action
Supporting package for kuka_motion. Defines the ROS action and message types needed.

### kuka_moveit_config
Includes all necessary config files for moveit.