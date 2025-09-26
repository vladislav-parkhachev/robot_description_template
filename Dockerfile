ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-core

# Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR /robot_description_ws/src

COPY robot_description/package.xml robot_description/

WORKDIR /robot_description_ws

# Install dependencies from package.xml
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /robot_description_ws/src
# Copy package
COPY robot_description robot_description

WORKDIR /robot_description_ws
# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Source ROS and workspace on start
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /robot_description_ws/install/setup.bash && exec \"$@\"", "--"]

# Default command (launch RViz with robot view)
CMD ["ros2", "launch", "robot_description", "robot_view.launch.py"]