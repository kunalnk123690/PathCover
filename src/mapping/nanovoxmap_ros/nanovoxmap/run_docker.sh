#!/bin/bash
clear
docker build -t nano-vox-map -f .devcontainer/Dockerfile .
docker run -it --rm \
  --net=host \
  --pid=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/input:/dev/input \
  -v /dev/dri:/dev/dri \
  -v $(pwd):/home/nanovox_map/nanovox_map_ws \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --runtime=nvidia \
  --gpus=all \
  --privileged \
  nano-vox-map \
  bash
