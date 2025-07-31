# ROS2 Dockerised Workspace with Moveit2 

## Getting Started

### 1. Clone the Repo

``` sh 
git clone --recurse-submodules https://github.com/Abhishek8857/ros2.git
```

If you’ve already cloned the repository without submodules, you can initialize and update the submodules like this:

```sh
git submodule update --init --recursive
```

### 2. Build docker image

Build the Docker image using the provided Dockerfile. This command must be run from the root of the repository where the Dockerfile is located:

```sh
bash docker_build.sh
```
This will create a Docker image with the default name from the file, which includes the ROS 2 environment and the Moveit2 and Moveit Task Constructor  package. You can change the name of the image by editing the docker_build.sh file


**The build process may take some time, especially on systems with lower RAM storage. If you have enough RAM storage, you can remove the MAKEFLAGS command form the DockerFile.**


### 3. Run the Docker Container

Once the Docker image is built, you can run the container interactively using:

```sh
cd docker_run/
bash docker_run.sh
```