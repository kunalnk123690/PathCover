#!/bin/bash
clear
docker system prune
# docker system prune -af --volumes

# --- GPU detection ----------------------------------------------------------
# The image builds and runs either way: nanovoxmap compiles its CUDA kernels
# only when nvcc is present, and falls back to the CPU path at runtime when no
# device is visible. So probe for a usable GPU and only then ask Docker for one
# -- passing --runtime=nvidia on a host without the NVIDIA container toolkit is
# a hard failure, not a graceful degradation.
GPU_ARGS=()
INSTALL_CUDA=${INSTALL_CUDA:-auto}

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
   && docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  echo "[run_docker] NVIDIA GPU + container runtime detected -- enabling CUDA"
  GPU_ARGS=(
    --runtime=nvidia
    --gpus=all
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
  )
  [ "$INSTALL_CUDA" = "auto" ] && INSTALL_CUDA=true
else
  echo "[run_docker] no NVIDIA GPU or container runtime -- running CPU-only"
  [ "$INSTALL_CUDA" = "auto" ] && INSTALL_CUDA=false
fi

# Device passthrough only for nodes that actually exist on this host.
DEV_ARGS=()
[ -d /dev/dri ]   && DEV_ARGS+=(-v /dev/dri:/dev/dri)
[ -d /dev/input ] && DEV_ARGS+=(-v /dev/input:/dev/input)

# INSTALL_CUDA=true forces the toolkit into the image even on a CPU-only build
# host (useful when the image is built here but run elsewhere); false skips the
# download.
# --network=host builds on the host's network stack instead of the default
# bridge. On a VPN (e.g. Cisco AnyConnect, whose cscotun0 runs a 1390-byte MTU)
# the 1500-byte bridge black-holes large packets: TCP connects fine, then apt's
# .deb downloads stall and time out mid-transfer. Sharing the host stack
# inherits the tunnel's MTU, so downloads complete. Harmless off-VPN.
docker build --network=host -t ros-noetic-pathcover \
  --build-arg INSTALL_CUDA="$INSTALL_CUDA" \
  -f .devcontainer/Dockerfile .

docker run -it --rm \
  --net=host \
  --pid=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "${DEV_ARGS[@]}" \
  -v $(pwd):/home/PathCover/PathCover_ws \
  "${GPU_ARGS[@]}" \
  --privileged \
  ros-noetic-pathcover \
  bash
