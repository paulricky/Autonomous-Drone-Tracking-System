FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential clang clang-format clang-tidy cmake gdb git python3-pip python3-pytest python3-venv \
    python3-rosdep python3-vcstool python3-setuptools ros-dev-tools \
    ros-jazzy-diagnostic-updater ros-jazzy-image-transport ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros ros-jazzy-rviz2 ros-jazzy-rosbag2 \
    libopencv-dev v4l-utils \
    ros-jazzy-camera-info-manager && rm -rf /var/lib/apt/lists/*
RUN rosdep init 2>/dev/null || true
WORKDIR /workspaces/robotics_ws
COPY requirements-ros.txt /tmp/requirements-ros.txt
RUN python3 -m venv --system-site-packages /opt/robotics-venv && \
    /opt/robotics-venv/bin/python -m pip install \
      --no-cache-dir \
      -r /tmp/requirements-ros.txt
ENV PATH="/opt/robotics-venv/bin:${PATH}"
ENV ROS_DISTRO=jazzy
COPY src/ /tmp/robotics-src/
RUN rosdep update && apt-get update && \
    rosdep install --from-paths /tmp/robotics-src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/* /tmp/robotics-src
CMD ["bash"]
