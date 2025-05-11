#!/bin/bash
DOCKER_BUILDKIT=1 docker build -t ros2-foxglove-bridge .
docker run --network host --rm -it ros2-foxglove-bridge
