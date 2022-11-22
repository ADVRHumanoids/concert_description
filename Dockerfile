FROM arturolaurenzi/xbot2_focal_base_nvidia:latest
USER user
SHELL ["/bin/bash", "-ic"]

WORKDIR /home/user
RUN mkdir -p forest_ws/ros_src 
COPY --chown=user:user modular_cartesio forest_ws/ros_src/modular_description/modular_cartesio
COPY --chown=user:user modular_examples forest_ws/ros_src/modular_description/modular_examples
COPY --chown=user:user modular_gazebo forest_ws/ros_src/modular_description/modular_gazebo
COPY --chown=user:user modular_resources forest_ws/ros_src/modular_description/modular_resources
COPY --chown=user:user modular_xbot2 forest_ws/ros_src/modular_description/modular_xbot2

# HACK we copy modular for now, since it's not available open source 
# NOTE: right now, external users can't build this docker image because of this!
COPY --chown=user:user modular forest_ws/src/modular

COPY --chown=user:user setup-docker.bash .

RUN echo '[0] will setup docker'
RUN bash -i setup-docker.bash

# set ownership to user for the whole home folder
RUN chown -R user .

# change user, copy start script (launches gazebo and gzweb)
USER user