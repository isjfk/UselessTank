# Installtion

## Install Ubuntu Mate in Raspberry Pi

## Install ROS packages
  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update

  sudo apt install ros-melodic-desktop
  sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo apt install ros-melodic-tf
  sudo apt install ros-melodic-navigation
  sudo apt install ros-melodic-slam-gmapping
  sudo apt install ros-melodic-rviz
  sudo apt install ros-melodic-joy
  sudo apt install ros-melodic-teleop-twist-joy
  sudo apt install ros-melodic-teleop-twist-keyboard

  sudo rosdep init
  rosdep update
  ```

## Initialize ROS workspace
  ```bash
  source /opt/ros/melodic/setup.bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  ```
  Then add following lines into ~/.bashrc file:
  ```bash
  source ~/catkin_ws/devel/setup.bash

  export ROS_IP=`ifconfig wlan0 | grep "inet " | awk '{ print $2 }'`
  export ROS_MASTER_URI="http://${ROS_IP}:11311"

  alias ws='cd ~/catkin_ws'
  alias wssrc='cd ~/catkin_ws/src'
  alias wsmake='cd ~/catkin_ws && catkin_make && cd -'
  alias ta='cd ~/catkin_ws/src/tank_agent/src'
  alias tarun='rosrun tank_agent tank_agent_node'
  alias tanksetup='roslaunch tank_2dnav tank_setup.launch'
  alias tanknav='roslaunch tank_2dnav tank_nav.launch'
  alias tank='roslaunch tank_2dnav tank.launch'
  alias tankds4='roslaunch tank_remote ds4.launch'
  ```

## Install 3rd packages
  ```bash
  sudo apt install gstreamer1.0-plugins-base-apps
  sudo apt install python-pip
  pip install Flask
  pip install Flask-Cors
  pip install rospkg
  ```

# Configuration

## Pairing PS4 joystick to Ubuntu Mate
  Pair with Ubuntu desktop tool "Bluetooth Manager".  
  To start, hold "SHARE" and "PS" button in PS4 joystick until indicator flashes.  
  To test the joystick, run:
  ```bash
  sudo apt install joystick
  jstest /dev/input/js0
  ```
