FROM ros:humble-ros-base

RUN apt update && apt install -y python3-pip
RUN pip3 install rpi_ws281x adafruit-circuitpython-neopixel
RUN apt install -y python3-rpi.gpio

COPY move_forward /opt/move_forward
WORKDIR /opt/move_forward
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

