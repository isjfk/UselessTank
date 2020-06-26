#!/bin/bash

# Configure permissions for ttyS*
usermod -a -G dialout tank

# Update apt packages library
apt-get update

# Don't promot for restart service while apt-install
#echo '* libraries/restart-without-asking boolean true' | sudo debconf-set-selections

# Install sound packages
apt-get install -y gstreamer1.0-plugins-base-apps
apt-get install -y sox libsox-fmt-all

# Configure bluetooth to support XBox One Controller
echo 'options bluetooth disable_ertm=Y' >> /etc/modprobe.d/bluetooth.conf

# Install common packages
apt-get install -y vim

# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update

apt-get install -y ros-melodic-desktop-full
apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep init
rosdep update
#rosdep fix-permissions

apt-get install -y ros-melodic-navigation ros-melodic-slam-gmapping ros-melodic-tf ros-melodic-laser-filters ros-melodic-robot-localization
apt-get install -y ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard joystick

# Create ROS workspace
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# Install Python & Flask
apt-get install -y python-pip
# Fix pip SSLError issue
python -m pip install --trusted-host pypi.python.org --trusted-host files.pythonhosted.org --trusted-host pypi.org --upgrade pip
pip install rospkg pyyaml
pip install Flask Flask-Cors

# Install NodeJS
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
apt-get install -y nodejs
apt-get install -y npm

# Create package directory
mkdir -p ~/package

# Copy resource files
cp -R ${SCRIPT_DIR}/resource/. ~/

# Fix file permissions caused by sudo
chown -R tank:tank ~

reboot
