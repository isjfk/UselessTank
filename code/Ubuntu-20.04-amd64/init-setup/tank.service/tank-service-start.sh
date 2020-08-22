#!/bin/bash

# This script is called by tank.service to start tank ROS nodes on system startup.

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
TANK_DIR=$(cd ~; pwd)

source $TANK_DIR/tank.bashrc

if [[ -z "$TANK_MODEL" ]] || [[ -z "$TANK_MOTOR_MODEL" ]]; then
    echo "ERROR: Please set TANK_MODEL & TANK_MOTOR_MODEL in $TANK_MODEL_CFG_FILE according to tank setup!"
    exit 11
fi

if [ -z "$ROS_IP" ]; then
    echo "ERROR: Failed to start tank ROS service, cannot detect IP of the tank WiFi!"
    exit 12
fi

echo "[tank] ROS_IP: $ROS_IP"
echo "[tank] Starting ROS nodes..."
roslaunch tank_2dnav tank.launch
