# Docker for ur3 repo
# ros-melodic-base, gazebo9, gps, python libraries 
# Python 3 version

FROM nvidia/cuda:11.4.2-devel-ubuntu20.04 AS base
# FROM tensorflow/tensorflow:latest-gpu-py3
# LABEL maintainer Cristian Beltran "beltran@hlab.sys.es.osaka-u.ac.jp"

#### Use an official ROS runtime as a parent image
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    # apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
# RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update \
    && apt-get install -y curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    && rm -rf /var/lib/apt/lists/*

# install universal robot ros packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-spacenav-node \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-eigen-conversions \
    # install catkin
    ros-$ROS_DISTRO-catkin \
    python3-catkin-tools \
    # Install Numpy Boost
    libboost-dev \
    libboost-python-dev \
    libboost-system-dev \
    libboost-all-dev \
    libatlas-base-dev \
    libprotobuf-dev \
    protobuf-compiler \
    # python dependencies
    python3-setuptools \
    python3-tk \
    python3-numpy \
    # utils
    locate \
    aptitude \
    vim htop tmux \
    curl wget \
    tk \
    spacenavd \
    && rm -rf /var/lib/apt/lists/*

### Gazebo ###
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

ENV DEBIAN_FRONTEND noninteractive

# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    binutils \
    mesa-utils \
    x-window-system \
    gazebo11 \
    libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

ADD ./docker/orocos_kinematics_dynamics.tar.xz /root/

# Install SIP 4.19.8
ADD ./docker/sip-4.19.8.tar.gz /root/

RUN /bin/bash -c "cd ~/ \
               && cd sip-4.19.8 \
               && python3 configure.py \
               && make -j4 && make install"

# Install PyKDL
RUN apt update && apt -y install libeigen3-dev && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "cd ~/orocos_kinematics_dynamics/orocos_kdl \
               && mkdir build && cd build \
               && cmake -DCMAKE_BUILD_TYPE=Release .. \
               && make -j4 && make install"

RUN /bin/bash -c "cd ~/orocos_kinematics_dynamics/python_orocos_kdl \
               && mkdir build && cd build \
               && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  -DPYTHON_LIBRARY=$(python3 -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") -DPYTHON_VERSION=3 .. \
               && make -j4"

COPY ./docker/bashrc /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc

### Ros Workspace ###
# Set up the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
               && mkdir -p ~/ros_ws/src \
               && cd ~/ros_ws/ \
               && catkin init"

RUN apt update && apt install -y git

# Installing repo required for homework
RUN /bin/bash -c "cd ~/ros_ws/src \
               && git clone https://bitbucket.org/traclabs/trac_ik.git \
               && git clone -b noetic-devel https://github.com/cambel/ur3.git ros_ur3 \
               && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git universal_robots_ros_driver \
               && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git universal_robot"

# Updating ROSDEP and installing dependencies
RUN cd ~/ros_ws \
    && rosinstall ~/ros_ws/src /opt/ros/$ROS_DISTRO src/ros_ur3/dependencies.rosinstall \
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONIOENCODING UTF-8
RUN /bin/bash -c "cd ~/ros_ws/src/gazebo_ros_link_attacher \
                  && cd ~/ros_ws/src/robotiq \
                  && git fetch origin && git checkout $ROS_DISTRO-devel"

## Python libraries ##
RUN apt update && apt install -y python3-pip
RUN python3 -m pip install pip --upgrade && \
    pip3 install matplotlib spicy protobuf pyyaml pyquaternion rospkg \
    lxml tqdm catkin-pkg empy PyVirtualDisplay defusedxml gym psutil pyprind

# Compiling ros workspace
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
#                && cd ~/ros_ws/ \
#                && rm -rf build \
#                && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DPYTHON_VERSION=3"

################################################
# Custom python libs
################################################
RUN apt-get update && apt-get -y upgrade && rm -rf /var/lib/apt/lists/*

RUN pip install Cython cpprb

# ur_ikfast
RUN /bin/bash -c "cd ~/ \
               && mkdir pylibs && cd pylibs \
               && git clone https://github.com/cambel/ur_ikfast.git \
               && cd ur_ikfast && pip install -e ."

# fix for sip
RUN rm /usr/lib/python3/dist-packages/sip.cpython-38-x86_64-linux-gnu.so

RUN pip install xmltodict imageio IPython

## Intel Realsense ROS packages
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

##### Easy hand-eye calibration libs
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-apriltag-ros \
    ros-$ROS_DISTRO-handeye \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-multiplot \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-runtime \
    ros-$ROS_DISTRO-moveit-python \
    ros-$ROS_DISTRO-moveit-resources \
    && rm -rf /var/lib/apt/lists/*

# Workaround for python3 + tf2
RUN pip install --extra-index-url https://rospypi.github.io/simple/ tf2_ros --ignore-installed
RUN pip install --upgrade pip

RUN /bin/bash -c "cd ~/ros_ws/src/robotiq \
                  && git pull \
                  && cd ../ros_ur3 \
                  && git pull"


RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
                && cd ~/ros_ws/src \
                && git clone -b $ROS_DISTRO-devel  https://github.com/ros/geometry2.git && \
                git clone https://github.com/eric-wieser/ros_numpy && \
                git clone https://github.com/JenniferBuehler/gazebo-pkgs"

# RUN source /opt/ros/$ROS_DISTRO/setup.bash \
#     && cd ~/ros_ws \
#     && wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/$ROS_DISTRO-devel/moveit.rosinstall \
#     && wstool update -t src

# RUN cd ~/ros_ws \
#     && apt-get update \
#     && rosdep update \
#     && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

# Compiling ros workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
               && cd ~/ros_ws/ \
               && rm -rf build \
               && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_VERSION=3"

RUN mkdir -p ~/.gazebo
RUN git clone https://github.com/osrf/gazebo_models ~/.gazebo/models
RUN pip install netifaces

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# setup environment
EXPOSE 11345

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc

RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export PYTHONPATH=/root/orocos_kinematics_dynamics/python_orocos_kdl/build:$PYTHONPATH' >> ~/.bashrc
RUN echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros_ws/src/ros-universal-robots/ur3_gazebo/models/' >> ~/.bashrc

### Build ROS-supported environment
FROM base as ros

USER root
ENV HOME=/root

COPY ./deep_grasp_vgu ${HOME}/ros_ws/src/deep_grasp_vgu

RUN /bin/bash -c "  cd $HOME/ros_ws/src/deep_grasp_vgu && \
                    git clone https://github.com/haotruongnhat/deep_grasp_msgs"

# Install torch
COPY ./wheels/torch-1.10.0+cu111-cp36-cp36m-linux_x86_64.whl $HOME/torch-1.10.0+cu111-cp36-cp36m-linux_x86_64.whl
RUN python3 -m pip install $HOME/torch-1.10.0+cu111-cp36-cp36m-linux_x86_64.whl
RUN python3 -m pip install opencv-python

RUN /bin/bash -c "cd $HOME/ros_ws/src  && \
                git clone -b melodic-devel  https://github.com/ros/geometry2.git && \
                git clone https://github.com/eric-wieser/ros_numpy"

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
               && cd ~/ros_ws \
               && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 \
                                -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
                                -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
                                -DPYTHON_VERSION=3"
