ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \ 
    LC_ALL=C.UTF-8 \
    ROS_DOMAIN_ID=0

# Remove old ROS 2 keys and source lists
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Fix expired ROS GPG key
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

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
    python3-rosinstall-generator \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    python3-pytest-cov \
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libacl1-dev \
    libignition-common4-dev \
    ros-humble-filters \
    ros-humble-rmw-cyclonedds-cpp \
    ros-dev-tools \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ign-ros2-control \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ackermann-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# Setup locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install Ignition Fortress (Gazebo 6)
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && apt-get install -y ignition-fortress libignition-gazebo6-dev && \
    rm -rf /var/lib/apt/lists/*

# Fix missing update
RUN apt-get update --fix-missing -y

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

# Copy the entire colcon_ws  and overlay_ws directory with the submodule into the Docker image
COPY colcon_ws/ /colcon_ws/
WORKDIR /colcon_ws/src/

WORKDIR /colcon_ws/

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Install dependencies
# RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace with resource management
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --symlink-install --executor sequential

# Copy entrypoint scripts and make them executable
COPY entrypoint_scripts/ /entrypoint_scripts/
RUN chmod +x /entrypoint_scripts/*.sh

# Copy contents in overlay ws
COPY overlay_ws/ /overlay_ws/
WORKDIR /overlay_ws/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-clean-cache --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entry point   
CMD ["/bin/bash"]
