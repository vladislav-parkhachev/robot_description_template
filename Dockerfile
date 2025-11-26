ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update && \
    apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    cmake && \
    rosdep init && \
    rosdep update

WORKDIR /robot_description_ws/src

COPY robot_description/package.xml robot_description/

WORKDIR /robot_description_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /robot_description_ws/src

COPY robot_description robot_description

WORKDIR /robot_description_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

ENV PROMPT_COMMAND="source /opt/ros/${ROS_DISTRO}/setup.bash"
CMD ["bash"]