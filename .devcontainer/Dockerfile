FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential clang clang-format clang-tidy cmake gdb git python3-pip python3-pytest \
    python3-ruff python3-rosdep python3-vcstool ros-dev-tools \
    ros-jazzy-diagnostic-updater ros-jazzy-image-transport ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros ros-jazzy-rviz2 ros-jazzy-rosbag2 \
    libopencv-dev && rm -rf /var/lib/apt/lists/*
RUN rosdep init 2>/dev/null || true
WORKDIR /workspaces/ECE3162
COPY requirements-ros.txt /tmp/requirements-ros.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements-ros.txt
ENV ROS_DISTRO=jazzy
CMD ["bash"]
