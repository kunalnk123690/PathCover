# Use a base image suitable for C development. Ubuntu is a good choice.
FROM nvidia/cuda:13.0.0-devel-ubuntu22.04

ARG USERNAME=nanovox_map
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt update \
    && apt install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt update && apt upgrade -y

# --- FIX FOR PCL / TZDATA HANGING ---
# 1. Pre-define your desired timezone 
ENV TZ=Etc/UTC
# 2. Instruct apt to run entirely without user configuration prompts
ARG DEBIAN_FRONTEND=noninteractive
# ------------------------------------

# Install build essential, development tools, and PCL
RUN apt install -y --no-install-recommends \
    build-essential \
    git \
    ninja-build \
    wget \
    pkg-config \
    ca-certificates \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y lsb-release gnupg curl
RUN apt install -y bash bash-completion
RUN apt install -y python3-pip libeigen3-dev

RUN pip completion --bash >> /home/$USERNAME/.bashrc
RUN pip install --upgrade pip
RUN pip install cmake

# ENV SHELL /bin/bash
RUN echo "export PATH=$PATH:/usr/local/bin:$HOME/.local/bin" >> /home/$USERNAME/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> /home/$USERNAME/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
