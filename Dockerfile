# FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04 as nvidia
FROM ros:noetic-ros-base-focal

# COPY --from=nvidia /usr/local /usr/local
# COPY --from=nvidia /etc/ld.so.conf.d/nvidia.conf /etc/ld.so.conf.d/nvidia.conf
# ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

ENV rosdistro noetic

# install all dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \ 
sudo file libyaml-cpp-dev build-essential cmake cmake-curses-gui git wget vim \
libmatio-dev \
ros-${rosdistro}-robot ros-${rosdistro}-interactive-markers ros-${rosdistro}-tf2-eigen ros-${rosdistro}-rviz ros-${rosdistro}-moveit-core \ 
gazebo11 libgazebo11-dev \ 
libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial \
qt5-default qttools5-dev qtquickcontrols2-5-dev qtdeclarative5-dev  ros-${rosdistro}-rosmon

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${rosdistro}-gazebo-ros-pkgs

# create a regular user
RUN useradd -ms /bin/bash user
RUN adduser user sudo
RUN echo 'user:user' | chpasswd

WORKDIR /home/user

# install qt5 charts and graphics drivers
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \ 
libqt5charts5-dev libgl1-mesa-glx libgl1-mesa-dri python3-pip

# install python modules for backtrace pretty printer
RUN pip3 install parse ansicolors notebook hhcm-forest

# install xbot (note: experimental mode !!)
RUN sh -c 'echo "deb http://xbot.cloud/xbot2-experimental/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-experimental.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | apt-key add -
RUN apt-get update && apt install -y xbot2_desktop_full 


# set ownership to user for the whole home folder
RUN chown -R user .

# change user, copy start script (launches gazebo and gzweb)
USER user
# COPY --chown=user:user docker/start.sh .
# COPY --chown=user:user docker/build-examples.sh .
RUN bash -c "echo source /opt/ros/noetic/setup.bash >> /home/user/.bashrc"
RUN bash -c "echo source /opt/xbot/setup.sh >> /home/user/.bashrc"
RUN bash -c "echo alias notebook_docker=\'jupyter notebook --no-browser --ip=0.0.0.0\' >> /home/user/.bashrc"

WORKDIR /home/user
RUN mkdir -p forest_ws/ros_src 
COPY modular_cartesio forest_ws/ros_src/modular_description/modular_cartesio
COPY modular_examples forest_ws/ros_src/modular_description/modular_examples
COPY modular_gazebo forest_ws/ros_src/modular_description/modular_gazebo
COPY modular_resources forest_ws/ros_src/modular_description/modular_resources
COPY modular_xbot2 forest_ws/ros_src/modular_description/modular_xbot2

COPY setup-docker.bash .
RUN bash -i setup-docker.bash