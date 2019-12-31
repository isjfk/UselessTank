#!/bin/bash

apt-get update

# Configure permissions for ttyS*
sed -r -i --follow-symlinks 's/(.*)console=serial[0-9]+,[0-9]+[[:blank:]](.*)/\1\2/' /boot/cmdline.txt
sed -r -i --follow-symlinks 's/(.*)console=ttyS[0-9]+,[0-9]+[[:blank:]](.*)/\1\2/' /boot/cmdline.txt
echo 'enable_uart=1' >> /boot/config.txt
usermod -a -G dialout tank

# Don't promot for restart service while apt-install
echo '* libraries/restart-without-asking boolean true' | sudo debconf-set-selections

# Install common packages
apt-get install -y vim

# Install sound packages & configure sound system
apt-get install -y gstreamer1.0-plugins-base-apps
apt-get install -y sox libsox-fmt-all
echo 'set-default-sink alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo' >> /etc/pulse/default.pa
echo 'set-default-source alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo.monitor' >> /etc/pulse/default.pa

# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update

apt-get install -y ros-melodic-desktop
apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep init
rosdep update

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
pip install rospkg pyyaml
pip install Flask Flask-Cors

# Install NodeJS
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
apt-get install -y nodejs

# Create package directory
mkdir -p ~/package

# Copy resource files
cp -R ${SCRIPT_DIR}/resource/. ~/

# Fix file permissions caused by sudo
chown -R tank:tank ~

reboot
