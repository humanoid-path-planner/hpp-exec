# hpp-planning: ROS2 Jazzy + HPP + Gazebo for motion planning
#
# Build:  docker build --build-arg DOCKER_USER=$(id -u) --build-arg DOCKER_GROUP=$(id -g) -t hpp-planning .
# Run:    ./run.sh
#
# On first run inside the container:
#   cd ~/devel/src && make all
#
FROM ros:jazzy

# robotpkg repository (HPP C++ packages for Ubuntu 24.04 / Noble)
RUN apt-get update -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -qqy curl \
 && mkdir -p /etc/apt/keyrings \
 && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc \
 && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" \
    > /etc/apt/sources.list.d/robotpkg.list \
 && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/wip/packages/debian/pub noble robotpkg" \
    >> /etc/apt/sources.list.d/robotpkg.list

# HPP C++ packages from robotpkg (base deps for building hpp-manipulation + hpp-python)
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    robotpkg-py312-pinocchio \
    robotpkg-py312-hpp-manipulation-corba \
    robotpkg-py312-example-robot-data \
    python3-numpy \
    python-is-python3

# Build tools (for hpp-manipulation + hpp-python from source)
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    git cmake build-essential \
    libboost-python-dev libboost-dev \
    python3-dev

# ROS2 control packages
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    ros-jazzy-control-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2controlcli \
    python3-colcon-common-extensions

# Visualization (rqt + rviz2)
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-topic \
    ros-jazzy-rviz2

# Gazebo Harmonic + ROS2 bridge
RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-gz-plugin-vendor \
    ros-jazzy-sdformat-urdf \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    python3-yaml \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create user with host UID/GID to avoid permission issues on mounted volumes
ARG DOCKER_USER=1000
ARG DOCKER_GROUP=1000
RUN groupadd -g ${DOCKER_GROUP} user 2>/dev/null || true \
 && useradd -m -s /bin/bash -u ${DOCKER_USER} -g ${DOCKER_GROUP} user

# Franka robot packages (built from source, not available via apt)
# Build in /opt so it's not shadowed by volume mounts
WORKDIR /opt/franka_ws
RUN mkdir -p src

RUN cd src && \
    git clone --depth 1 https://github.com/frankarobotics/franka_description.git && \
    git clone --branch jazzy --depth 1 https://github.com/frankarobotics/franka_ros2.git

RUN apt-get update && apt-get install -y python3-rosdep && \
    (rosdep init 2>/dev/null || true) && \
    rosdep update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN chown -R ${DOCKER_USER}:${DOCKER_GROUP} /opt/franka_ws /home/user
USER user

RUN cd /opt/franka_ws && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-up-to \
        franka_description \
        franka_gazebo_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo 'source /home/user/devel/config.sh 2>/dev/null' >> /home/user/.bashrc \
 && echo 'source /opt/ros/jazzy/setup.bash' >> /home/user/.bashrc \
 && echo 'source /opt/franka_ws/install/setup.bash 2>/dev/null' >> /home/user/.bashrc

WORKDIR /home/user/devel
CMD ["bash"]
