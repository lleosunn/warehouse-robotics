FROM robomaster_bridge:latest
WORKDIR /opt/robomaster/robomaster_ros2_can

COPY teleop_twist_keyboard-dashing/ /opt/robomaster/robomaster_ros2_can/teleop_twist_keyboard-dashing/
RUN colcon build --packages-select teleop_twist_keyboard

RUN echo 'source /opt/robomaster/robomaster_ros2_can/install/setup.bash' >>  ~/.bashrc
RUN echo 'alias control="ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/robomaster_1/cmd_vel"' >>  ~/.bashrc
CMD ["/bin/bash"]