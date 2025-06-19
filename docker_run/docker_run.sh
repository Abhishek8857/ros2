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
        --name kuka \
        -v "$REPO_DIR:/kuka:rw" \
        -v $PARENT_DIR:/root/kuka_ws:rw \
        -w /overlay_ws \
        kuka:latest \
        /kuka/entrypoint_scripts/entrypoint_docker_run.sh
