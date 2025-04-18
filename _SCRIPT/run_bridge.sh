
docker run -d --rm --network host robomaster_bridge:latest /bin/bash -c ". install/setup.bash && ros2 launch robomaster_can_ros_bridge/launch/bridge.launch.py"
docker run -d --rm --network host -e JETSON_MODEL_NAME=JETSON_ORIN_NX --device /dev/i2c-7 --device /dev/gpiochip0 robomaster_ui:latest
