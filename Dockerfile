FROM ros:jazzy-ros-base-noble AS build
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-pip \
    libopencv-dev v4l-utils clang-format-18 \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY . .
RUN source /opt/ros/jazzy/setup.bash && \
    colcon list --base-paths src && \
    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ece3162_bringup diagnostics.launch.py"]
