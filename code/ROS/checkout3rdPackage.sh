#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

cd "$SCRIPT_DIR"
rm -rf rplidar_ros
git clone git@github.com:Slamtec/rplidar_ros.git

cd "$SCRIPT_DIR"
rm -rf realsense-ros
git clone git@github.com:IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`

cd "$SCRIPT_DIR"
rm -rf laser_filters
git clone git@github.com:isjfk/laser_filters.git
cd laser_filters/
git checkout laser_filter_sector

# Temprary packages
cd "$SCRIPT_DIR"
rm -rf joystick_drivers
git clone git@github.com:ros-drivers/joystick_drivers.git
cd "$SCRIPT_DIR"
rm -rf teleop_twist_joy
git clone git@github.com:ros-teleop/teleop_twist_joy.git