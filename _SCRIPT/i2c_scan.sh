#!/bin/bash

# Camera I2C Scanner Script
# Runs i2cdetect on all available buses

echo "=== Listing available I2C buses ==="
i2cdetect -l
echo

# Extract bus numbers from i2cdetect -l
buses=$(i2cdetect -l | awk '{print $1}' | sed 's/i2c-//')

echo "=== Scanning all I2C buses ==="
for bus in $buses; do
    echo
    echo "============================================== Scanning $bus"
    sudo i2cdetect -ry $bus
done

echo
echo "=== Scan Complete ==="
