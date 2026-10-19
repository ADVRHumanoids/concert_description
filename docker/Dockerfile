FROM arturolaurenzi/xbot2_focal_base_nvidia:latest

LABEL org.opencontainers.image.source="https://github.com/ADVRHumanoids/concert_description"

USER root
SHELL ["/bin/bash", "-ic"]

# Fix expired ROS key
RUN wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# add xbot2 source and keys
# base image's xbot.list points to the dead xbot.cloud, so replace it before the first apt-get update
RUN wget -qO - https://advrhumanoids.github.io/XBotBuild/KEY.gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/xbot2.gpg && \
    sudo sh -c 'echo "deb https://advrhumanoids.github.io/XBotBuild/ubuntu/focal ./" > /etc/apt/sources.list.d/xbot.list' && \
    sudo apt-get update && \
    sudo apt-get install -y wget gnupg curl gnupg2 ca-certificates lsb-release

USER user
SHELL ["/bin/bash", "-ic"]

WORKDIR /home/user

# create forest ws and use it to clone and install CONCERT's simulation package
RUN mkdir concert_ws
WORKDIR /home/user/concert_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc
RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag concert 

# upgrade pip and setuptools
RUN python3 -m pip install --upgrade pip setuptools wheel
# Modify PATH to include local bin
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# concert packages
RUN forest grow pybind11 --verbose --jobs 4 --pwd user
RUN forest grow concert_all --verbose --jobs 4 --pwd user

# a few usage tips..
RUN echo 'echo "USAGE:"' >> /home/user/.bashrc
RUN echo 'echo "run simulation....: mon launch concert_gazebo concert.launch"' >> /home/user/.bashrc
RUN echo 'echo "run monitoring gui: xbot2-gui"' >> /home/user/.bashrc
RUN echo 'echo "run CartesI/O IK..: mon launch concert_cartesio concert.launch xbot:=true gui:=true"' >> /home/user/.bashrc
RUN echo 'echo "run odometry......: mon launch concert_odometry concert_odometry.launch publish_ground_truth:=true gui:=true"' >> /home/user/.bashrc

RUN echo 'echo ""' >> /home/user/.bashrc

# set ownership to user for the whole home folder
RUN chown -R user .

# change user, copy start script (launches gazebo and gzweb)
USER user
