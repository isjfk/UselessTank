#!/bin/bash

# Configure permissions for ttyS*
usermod -a -G dialout tank

# Update apt packages library
apt-get update

# Don't promot for restart service while apt-install
#echo '* libraries/restart-without-asking boolean true' | sudo debconf-set-selections

# Install sound packages
apt-get install -y gstreamer1.0-plugins-base-apps sox libsox-fmt-all

# Configure bluetooth to support XBox One Controller
echo 'options bluetooth disable_ertm=Y' >> /etc/modprobe.d/bluetooth.conf

# Install common packages
apt-get install -y vim curl

# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update

apt-get install -y ros-noetic-desktop-full
apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
rosdep init
rosdep update
#rosdep fix-permissions

apt-get install -y ros-noetic-ddynamic-reconfigure

# Navigation packages
apt-get install -y ros-noetic-navigation ros-noetic-slam-gmapping ros-noetic-tf ros-noetic-laser-filters ros-noetic-robot-localization

# Joystick packages
apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard joystick

# Deepth camera related packages
apt-get install -y ros-noetic-pointcloud-to-laserscan

# Create ROS workspace
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd ~

# Install Python3 & Flask
apt-get install -y python3-pip
pip3 install rospkg pyyaml
pip3 install Flask Flask-Cors

# Install NodeJS
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
apt-get install -y nodejs

# Create package directory
mkdir -p ~/package

# Copy tank home files
cp -R ${SCRIPT_DIR}/tank/. ~/
# Copy startup service scripts
mv -R ~/tank.service ~/..

# Fix file permissions caused by sudo
chown -R tank:tank ~

reboot
