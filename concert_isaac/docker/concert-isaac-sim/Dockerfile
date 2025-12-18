FROM isaac-lab-base:latest

USER root

RUN apt update && apt install -y \
    mesa-utils

WORKDIR /workspace

SHELL ["bash", "-ic"]

RUN PB_REL="https://github.com/protocolbuffers/protobuf/releases" && \
    curl -LO $PB_REL/download/v30.2/protoc-30.2-linux-x86_64.zip && \
    unzip protoc-30.2-linux-x86_64.zip -d /usr && \
    rm protoc-30.2-linux-x86_64.zip && \
    /workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install zmq && \
    /workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install pygame && \
    /workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install evdev

