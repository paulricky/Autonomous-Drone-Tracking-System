# Build natively on the Pi. The Jazzy Noble base image is multi-architecture.
FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG TARGETARCH
RUN test "$TARGETARCH" = "amd64" -o "$TARGETARCH" = "arm64"
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-rosdep \
    python3-numpy python3-scipy python3-yaml python3-matplotlib \
    libopencv-dev v4l-utils \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/robotics/workspace
COPY src/ src/
RUN rosdep init 2>/dev/null || true; rosdep update && apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --base-paths src --install-base /opt/robotics/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN mkdir -p /var/lib/robotics/calibration /var/log/robotics
ENV ROS_DISTRO=jazzy
ENV ROS_LOG_DIR=/var/log/robotics
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && ros2 launch robotics_bringup diagnostics.launch.py"]
