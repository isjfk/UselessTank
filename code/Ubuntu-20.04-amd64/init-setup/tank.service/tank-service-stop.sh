#!/bin/bash

# This script is called by tank.service to start tank ROS nodes on system startup.

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
TANK_DIR=$(cd ~; pwd)
ROS_LOG_RETENTION_DAYS=30

source $TANK_DIR/tank.bashrc

echo "Stop ROS..."
while [ -n "`pgrep roslaunch`" ]; do
    killall roslaunch
    sleep 2
done

echo "Clear ROS logs earlier then $ROS_LOG_RETENTION_DAYS days..."
RM_COUNT=`find "$TANK_DIR/.ros/log" -maxdepth 1 -mtime +$ROS_LOG_RETENTION_DAYS | wc -l`
find "$TANK_DIR/.ros/log" -maxdepth 1 -mtime +$ROS_LOG_RETENTION_DAYS -exec rm -rf {} \;
echo "Clear $RM_COUNT ROS log files/dirs."
