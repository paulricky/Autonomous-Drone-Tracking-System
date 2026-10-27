FROM ros:jazzy-ros-base-noble AS build
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-pip python3-venv \
    libopencv-dev v4l-utils clang-format-18 \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
COPY requirements-ros.txt /tmp/requirements-ros.txt
RUN python3 -m venv --system-site-packages /opt/robotics-venv && \
    /opt/robotics-venv/bin/python -m pip install --no-cache-dir -r /tmp/requirements-ros.txt
ENV PATH="/opt/robotics-venv/bin:${PATH}"
WORKDIR /workspace
COPY . .
RUN rosdep init 2>/dev/null || true; rosdep update && apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/jazzy/setup.bash && \
    colcon list --base-paths src && \
    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robotics_bringup diagnostics.launch.py"]
