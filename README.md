# ROS2 Dockerised Workspace with Moveit2 

## Getting Started

For more details, see the [KUKA RSI Wiki](https://github.com/kroshu/kuka_drivers/wiki/2_KSS_RSI).

### 1. Clone the Repo

``` sh 
git clone --recurse-submodules https://github.com/Abhishek8857/ros2.git
```

- If you’ve already cloned the repository without submodules, you can initialize and update the submodules like this:

```sh
git submodule update --init --recursive
```

### 2. Build docker image

Build the Docker image using the provided Dockerfile. This command must be run from the root of the repository where the Dockerfile is located:

```sh
bash docker_build.sh
```
- This will create a Docker image with the default name from the file, which builds the packages in the `overlay_ws`. They include Moveit2.


**The build process may take some time, especially on systems with lower RAM storage. If you have enough RAM storage, you can remove the MAKEFLAGS command form the DockerFile.**


### 3. Run the Docker Container

Once the Docker image is built, you can run the container interactively using:

```sh
cd docker_run/
bash docker_run.sh
```

### 4. Launch the KUKA Driver 

```sh
ros2 launch kuka_bringup driver.launch.py
```
- This will start the 3 controllers `joint_state_broadcaster`, `joint_trajectory_controller` & `gpio_controller`. If you want to launch without the `gpio_controller`, use the `use_gpio:=false` as shown below while launching the driver.

```sh
ros2 launch kuka_bringup driver.launch.py use_gpio:=false
```

### 5. Configure the Robot Manager
```sh
ros2 lifecycle set robot_manager configure
```
- You should get `Transition sucessful` message on sucessful configuration. Now we need to activate the manager.

```sh
ros2 lifecycle set robot_manager activate
```

- After the activation, a timeout of 30 seconds is provided to launch the `rsi_joint_pos_12ms.src` file on the teachpendant. On successful activation, you will again get `Transition successful` message.



### Changes done in packages:
- `kuka_drivers` : 
    1. GPIO code has been uncommented to allow input/output commands for the TCP
    2. Timeout has been changed from 10 to 30 in `robot_manager_node_rsi_only.cpp` and `hardware_interface_rsi_only.cpp`
- `kuka_external_control_sdk`
    1. Added IP address of the PC in `rsi_ethernet.xml` 

## TODO's
- [x] Edit the URDF with the updated `kuka_robot_description` files
- [x] Create Moveit config files for the arm
- [x] Integrate the Omnimove platform with the arm
- [x] Integrate the LiDAR sensors with the platform
- [x] Modify the Platform URDF to make it possible to translate in X, Y and rotate in Z Axes
- [x] Test Lidar Sensors and confirm if they are recieving data


