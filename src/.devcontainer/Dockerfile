FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        sudo \
        software-properties-common \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        vim \
        nano \
        xauth \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# =========================================================
# ROS 2 CONTROL
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-control-msgs \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-trajectory-controller \
        ros-humble-diff-drive-controller \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# GAZEBO
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        ros-humble-gazebo-ros2-control \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# ROBOT DESCRIPTION / URDF / TF
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# NAVIGATION
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# SLAM
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-slam-toolbox \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# RVIZ
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# SENSOR SUPPORT
# =========================================================

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-laser-filters \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-image-transport \
        ros-humble-cv-bridge \
        ros-humble-camera-info-manager \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# CREATE USER
# =========================================================

RUN groupadd --gid $GID $USERNAME \
    && useradd \
        --uid $UID \
        --gid $GID \
        --create-home \
        --shell /bin/bash \
        $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# =========================================================
# WORKSPACE
# =========================================================

RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

WORKDIR /home/$USERNAME/ros2_ws

# =========================================================
# BASH CONFIG
# =========================================================

RUN echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# =========================================================
# ENTRYPOINT
# =========================================================

COPY .devcontainer/ros_entrypoint.sh /ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

USER $USERNAME

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]