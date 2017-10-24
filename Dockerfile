FROM ubuntu:xenial

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# set up environment
ENV DEBIAN_FRONTEND noninteractive

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot
RUN apt-get update
RUN echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted" | tee /etc/apt/sources.list.d/docker.list
RUN echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted" | tee /etc/apt/sources.list.d/docker.list
RUN echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse" | tee /etc/apt/sources.list.d/docker.list
RUN echo "deb-src http://security.ubuntu.com/ubuntu xenial-security main restricted" | tee /etc/apt/sources.list.d/docker.list
RUN apt-get -y install ros-kinetic-turtlebot-gazebo ros-kinetic-desktop-full ros-kinetic-turtlebot-rviz-launchers  ros-kinetic-warehouse-ros

# Set up environment
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc