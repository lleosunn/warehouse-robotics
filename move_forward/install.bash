#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

echo "Build docker container"
docker build . -t move_forward:latest

echo "Prepare system service and config files"
set +e # read returns 1 by default
read -r -d '' btn_service <<EOF
[Unit]
Description= Move Forward
After=docker.service
Requires=docker.service
After=systemd-networkd.service
Requires=systemd-networkd

[Service]
ExecStart=/usr/bin/docker run --rm --network host --hostname $(hostname) --privileged move_forward:latest /bin/bash -c ". install/setup.bash && ros2 run move_forward move_forward"
KillSignal=SIGINT
Restart=always

[Install]
WantedBy=multi-user.target
EOF

set -e

echo "Write systemd service"
echo "${btn_service}" > /etc/systemd/system/move_forward.service

echo "Enable and start systemd service"
systemctl enable move_forward
systemctl start move_forward

echo "Success! Reboot required."

