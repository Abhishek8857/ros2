ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Switch mirror if getting package metadata takes long
RUN sed -i 's|http://.*.ubuntu.com|	http://de.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list

# Update and install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools \
    ros-${ROS_DISTRO}-filters \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-ros2-control \
    && rm -rf /var/lib/apt/lists/*


# Setup locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Fix missing update
RUN apt-get update --fix-missing -y

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Copy the entire colcon_ws directory with the submodule into the Docker image
COPY colcon_ws/ /colcon_ws/

# Colcon workspace
WORKDIR /colcon_ws/src/

# Update package lists and import MoveIt repositories based on the specified ROS distribution
# RUN apt-get update && \
#     for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do \
#         vcs import < "$repo"; \
#     done && \
#     vcs import --skip-existing --input ros2_kortex/ros2_kortex.$ROS_DISTRO.repos && \
#     vcs import --skip-existing --input ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos && \
#     rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y

# Colcon workspace
# WORKDIR /colcon_ws/

# # Build the workspace with resource management
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     MAKEFLAGS="-j4 -l3" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 


# Copy entrypoint scripts and make them executable
COPY entrypoint_scripts/ /entrypoint_scripts/
RUN chmod +x /entrypoint_scripts/*.sh

# Copy contents in overlay ws
COPY overlay_ws/ /overlay_ws/
WORKDIR /overlay_ws/

RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys=warehouse_ros_mongo

RUN source /opt/ros/${ROS_DISTRO}/setup.bash  && \
    colcon build  \
   --event-handlers desktop_notification- console_cohesion- \
   --cmake-clean-first \
   --cmake-args -DCMAKE_BUILD_TYPE=Release 

# Entry point   
CMD ["/bin/bash"]

