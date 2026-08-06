# This is a Dockerfile for ros:noetic-desktop
FROM osrf/ros:noetic-desktop-full

ARG USERNAME=quadrotor
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user and add sudo support
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt update \
    && apt install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update and install common packages
RUN apt update && apt upgrade -y
RUN apt install -y bash bash-completion
RUN apt install -y build-essential git wget libeigen3-dev python3-pip libcdd-dev
RUN pip install --upgrade pip
RUN pip install --upgrade cmake casadi gpg

# Source ROS Noetic in .bashrc and add aliases for catkin build system
RUN echo "export PATH=$HOME/.local/bin:$PATH" >> /home/$USERNAME/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "alias SOURCE_WS='source devel/setup.bash'" >> /home/$USERNAME/.bashrc
RUN echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> /home/$USERNAME/.bashrc

# Install ROS Noetic packages
RUN apt update && apt install -y \
    ros-noetic-message-filters \
    ros-noetic-gazebo-ros \
    ros-noetic-velodyne \
    ros-noetic-velodyne-simulator \
    ros-noetic-hector-gazebo-plugins

# --- Optional CUDA toolkit --------------------------------------------------
# nanovoxmap builds its GPU raycast/ESDF kernels only when CMake finds a CUDA
# compiler, and falls back to the CPU path otherwise, so the toolkit is a build
# convenience rather than a hard dependency. Only nvcc and the runtime headers
# are installed (not the full ~5 GB cuda-toolkit metapackage): that is all
# find_package(CUDAToolkit) + CUDA::cudart need.
#
# Skip the download entirely with:  --build-arg INSTALL_CUDA=false
# The install is deliberately fail-soft -- no NVIDIA repo for this
# architecture, or no network, leaves a working CPU-only image.
ARG INSTALL_CUDA=true
# CUDA 12.x keeps minor-version compatibility, so binaries built here run on any
# driver >= 525.60 regardless of which 12.x the host driver reports.
ARG CUDA_VERSION=12-6
RUN if [ "$INSTALL_CUDA" = "true" ]; then \
      case "$(dpkg --print-architecture)" in \
        amd64) CUDA_REPO_ARCH=x86_64 ;; \
        arm64) CUDA_REPO_ARCH=sbsa ;; \
        *)     CUDA_REPO_ARCH= ;; \
      esac ; \
      if [ -n "$CUDA_REPO_ARCH" ] \
         && wget -q -O /tmp/cuda-keyring.deb \
              "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${CUDA_REPO_ARCH}/cuda-keyring_1.1-1_all.deb" \
         && dpkg -i /tmp/cuda-keyring.deb \
         && apt update \
         && apt install -y --no-install-recommends \
              cuda-nvcc-${CUDA_VERSION} \
              cuda-cudart-dev-${CUDA_VERSION} \
              cuda-cccl-${CUDA_VERSION} ; then \
        ln -sfn /usr/local/cuda-$(echo $CUDA_VERSION | tr '-' '.') /usr/local/cuda ; \
        rm -f /tmp/cuda-keyring.deb ; \
        echo "CUDA toolkit installed: $(/usr/local/cuda/bin/nvcc --version | tail -1)" ; \
      else \
        echo "WARNING: CUDA toolkit unavailable -- image will build nanovoxmap CPU-only" ; \
      fi ; \
    else \
      echo "INSTALL_CUDA=false -- skipping CUDA toolkit, nanovoxmap builds CPU-only" ; \
    fi

# nvcc on PATH so catkin_make's check_language(CUDA) finds it. Harmless when the
# toolkit was skipped: the paths simply do not exist and the CPU path is used.
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
# Requests driver passthrough when the container is started with the NVIDIA
# runtime; ignored entirely when it is not, so CPU-only hosts are unaffected.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Set the default user
USER $USERNAME
WORKDIR /home/$USERNAME

CMD ["/bin/bash"]