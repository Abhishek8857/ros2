#!/bin/bash

SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"
PARENT_DIR="$(realpath "${REPO_DIR}/..")"


xhost +
docker run \
		-it \
		--rm \
		--net=host \
		--pid=host \
		--ipc=host \
		--privileged \
        --gpus all \
        --runtime=nvidia \
		-v /dev:/dev \
		-v $HOME/.ros/log:/.ros/log \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--env RMW_IMPLEMENTATION=${rmw_cyclonedds_cpp} \
		--env DISPLAY=$DISPLAY \
        --name kinova_ops \
        -v "$REPO_DIR:/moveit:rw" \
        -v $PARENT_DIR:/root/moveit_ws:rw \
        -w /overlay_ws \
        moveit:latest \
        /moveit/entrypoint_scripts/entrypoint_docker_run.sh
