# Shared by noninteractive build shells, the entrypoint, and interactive shells.
# ROS and Forest setup scripts may reference unset variables: do not use nounset.
source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /home/user/env/bin/activate
if [[ -f "${CONCERT_WS}/setup.bash" ]]; then
    source "${CONCERT_WS}/setup.bash"
fi

# Forest's merged-prefix setup does not source individual ament package hooks.
# The camera plugin registers its Gazebo library search paths through this hook.
if [[ -f "${CONCERT_WS}/install/share/realsense_gazebo_plugin/local_setup.bash" ]]; then
    source "${CONCERT_WS}/install/share/realsense_gazebo_plugin/local_setup.bash"
fi
