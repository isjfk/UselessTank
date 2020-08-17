#!/bin/bash

# This script is called by tank.service to start tank ROS nodes on system startup.

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
TANK_DIR=$(cd ~; pwd)

source $TANK_DIR/tank.bashrc

cd ~/package/tank-guider
npm start
