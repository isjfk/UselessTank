# Install Ubuntu Desktop 18.04.4 LTS amd64

## Prepare the USB driver with Ubuntu image
Flash image "ubuntu-18.04.4-desktop-amd64.iso" into TF card by balenaEtcher.  
Boot computer with the USB driver and install Ubuntu.

## Configure WiFi
  Login into Ubuntu desktop, connect to WiFi in status bar of upper right conor.  
  Select "Protected EAP (PEAP)" in Authentication.  
  Select "No CA certificate is required".  
  Input you domain account in Username & Password.  
  Click "Connect".  

## Update Ubuntu to latest packages
  In Ubuntu open bash shell (Ctrl+Alt+T in desktop):
  ```bash
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get dist-upgrade
  ```

## Install openssh-server to enable SSH remote login
  In Ubuntu open bash shell (Ctrl+Alt+T in desktop):
  ```bash
  sudo apt-get install net-tools openssh-server
  ```

## Copy ssh private key
  Copy ssh private key files into ~/.ssh to enable ssh login by certification.

# Setup ROS automatically by script
  Transfer directory ./init-setup to Ubuntu ~/ directory by SFTP.  
  Login into Ubuntu bash shell.
  ```bash
  cd ~/init-setup
  chmod a+x *.sh
  sudo ./init-setup.sh
  ```
  After script finshed in success, it will reboot automatically.  
  If you got any errors occur during execute init-setup.sh, please execute rest of the steps manually. Otherwise there may redundance lines writen into some configuration files.

# Setup ROS manually
  In case you prefer the manual way.  
  If you execute init-setup.sh already then you should omit this section.

## Config serial port
  ```bash
  sudo usermod -a -G dialout `whoami`
  ```

## Install common packages
  ```bash
  sudo apt-get install vim
  sudo apt-get install gstreamer1.0-plugins-base-apps
  sudo apt-get install sox libsox-fmt-all
  ```

## Install ROS packages
  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt-get update

  sudo apt-get install ros-melodic-desktop
  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update

  sudo apt-get install ros-melodic-navigation ros-melodic-slam-gmapping ros-melodic-tf
  sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard joystick
  ```

## Initialize ROS workspace
  ```bash
  source /opt/ros/melodic/setup.bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  ```
  Then copy all files in project ./init-setup/resource directory into Ubuntu ~ directory.

## Install Python & Flask
  ```bash
  sudo apt-get install python-pip
  pip install rospkg pyyaml
  pip install Flask Flask-Cors
  ```

## Reboot
  ```bash
  sudo reboot
  ```

# Setup tank model
  Edit file ~/tank_model.env, uncomment lines that match the tank setup.

# Device Configuration

## Pairing XBox One joystick to Ubuntu
  In Ubuntu open "Settings -> Bluetooth".  
  To start pairing, hold "SHARE" and "PS" button in PS4 joystick until indicator flashes.  
  To test the joystick, run:
  ```bash
  jstest /dev/input/js0
  ```
