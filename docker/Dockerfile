# Override with an image built from ADVRHumanoids/xbot2_docker/noble/Dockerfile.
# Pin a tested tag or digest for release builds.
ARG BASE_IMAGE=hhcmhub/xbot2-noble-dev:latest
FROM ${BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]

ENV CONCERT_WS=/home/user/concert_ws \
    HHCM_FOREST_CLONE_DEFAULT_PROTO=https \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    BASH_ENV=/home/user/concert-docker-env.bash

COPY docker/environment.bash /home/user/concert-docker-env.bash
COPY docker/entrypoint.bash /usr/local/bin/concert-entrypoint
RUN chmod 755 /usr/local/bin/concert-entrypoint && \
    mkdir -p "$CONCERT_WS" && chown user:user "$CONCERT_WS"

USER user
WORKDIR /home/user/concert_ws

# Reuse the base image's system-site-packages venv, rather than installing
# Python packages into Ubuntu's externally managed system interpreter.
# The base owns the toolchain; check the build requirements without running apt.
RUN test "$ROS_DISTRO" = jazzy && \
    test -f /home/user/env/bin/activate && \
    for tool in c++ make cmake git; do \
        command -v "$tool" || { echo "Missing build tool: $tool" >&2; exit 1; }; \
    done && \
    python -c 'import sysconfig; from pathlib import Path; assert (Path(sysconfig.get_path("include")) / "Python.h").is_file(), "Base image is missing Python development headers"' && \
    python -c 'from importlib.metadata import version; from packaging.version import Version; assert Version(version("hhcm-forest")) >= Version("1.1.23")'

ARG RECIPES_REPO=https://github.com/ADVRHumanoids/multidof_recipes.git
ARG RECIPES_REF=ros2
ARG BUILD_JOBS=4

RUN forest init && \
    forest add-recipes "$RECIPES_REPO" --tag "$RECIPES_REF" --clone-protocol https

# Build this checkout, including uncommitted changes and the initialized
# concert_resources submodule. Only ROS 2 packages selected by the recipe build.
# Keep the dependency tree in the Forest recipes, rather than duplicating it
# here. Forest resolves concert_all and reuses installed XBot2 binaries.
COPY --chown=user:user . src/concert_description/
# BASH_ENV already loads XBot2 for build shells. Source it explicitly here
# before Forest resolves dependencies, then restore the workspace overlay.
# Fail if the binary installation is unavailable instead of silently building it.
RUN source /opt/xbot/setup.sh && \
    source "$CONCERT_WS/setup.bash" && \
    command -v xbot2-core && \
    test -f src/concert_description/concert_resources/CMakeLists.txt && \
    forest grow concert_all --verbose --jobs "$BUILD_JOBS" --clone-protocol https && \
    echo 'source /home/user/concert-docker-env.bash' >> /home/user/.bashrc

# Resolve the installed launch dependencies and executables before shipping.
RUN for package in concert_gazebo concert_xbot2 modular_resources \
        realsense_gazebo_description ros_gz_sim ros_gz_bridge ros_gz_image xacro; do \
        ros2 pkg prefix "$package"; \
    done && \
    for tool in xbot2-core xbot2_gui_server xbot2_gui; do \
        command -v "$tool" || { echo "Missing runtime executable: $tool" >&2; exit 1; }; \
    done && \
    python -c 'import modular.URDF_writer'

ENTRYPOINT ["/usr/local/bin/concert-entrypoint"]
CMD ["/bin/bash"]
