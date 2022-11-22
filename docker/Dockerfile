FROM arturolaurenzi/xbot2_focal_base_nvidia:latest
USER user
SHELL ["/bin/bash", "-ic"]

WORKDIR /home/user

# create forest ws and use it to clone and install CONCERT's simulation package
RUN mkdir concert_ws
WORKDIR /home/user/concert_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc
RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master 

# steering/rolling modules for ik
RUN forest grow centauro_cartesio -j 4

# concert simulation package
RUN forest grow concert_description

# a few usage tips..
RUN echo 'echo "USAGE:"' >> /home/user/.bashrc
RUN echo 'echo "run simulation....: mon launch concert_gazebo concert.launch"' >> /home/user/.bashrc
RUN echo 'echo "run monitoring gui: xbot2-gui"' >> /home/user/.bashrc
RUN echo 'echo "run CartesI/O IK..: mon launch concert_cartesio concert.launch xbot:=true gui:=true"' >> /home/user/.bashrc
RUN echo 'echo ""' >> /home/user/.bashrc

# set ownership to user for the whole home folder
RUN chown -R user .

# change user, copy start script (launches gazebo and gzweb)
USER user