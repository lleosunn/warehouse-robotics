
# Grab the official slim ROS 2 base
FROM ros:humble-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# install only foxglove-bridge and clean up apt cache
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/*

# default entrypoint: source ROS and launch the bridge
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml"]

