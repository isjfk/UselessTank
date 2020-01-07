# Install Ubuntu Mate in Raspberry Pi

## Prepare the TF card with Ubuntu Mate image
Flash image "ubuntu-mate-18.04.2-beta1-desktop-armhf+raspi-ext4.img" into TF card by balenaEtcher.  
Boot Raspberry Pi with the TF card.

## Configure WIFI
  Login into Ubuntu desktop.  
  Select "Protected EAP (PEAP)" in Authentication.  
  Select "No CA certificate is required".  
  Input you domain account in Username & Password.  
  Click "Connect".  

## Enable SSH remote login
  The SSH server is installed but not enabled by default. To enabled the SSH server, open Ubuntu bash shell (Ctrl+Alt+T in desktop):
  ```bash
  sudo dpkg-reconfigure openssh-server
  sudo systemctl enable ssh
  sudo systemctl start ssh
  ```

# Setup ROS automatically by script
  Transfer directory /code/UbuntuMate/init-setup to Ubuntu ~/ directory by SFTP.  
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
  sudo sed -r -i --follow-symlinks 's/(.*)console=serial[0-9]+,[0-9]+[[:blank:]](.*)/\1\2/' /boot/cmdline.txt
  sudo sed -r -i --follow-symlinks 's/(.*)console=ttyS[0-9]+,[0-9]+[[:blank:]](.*)/\1\2/' /boot/cmdline.txt
  sudo usermod -a -G dialout `whoami`
  ```

## Install common packages
  ```bash
  sudo apt-get install vim
  ```

## Install & configure audio
  ```bash
  sudo apt-get install gstreamer1.0-plugins-base-apps
  sudo apt-get install sox libsox-fmt-all
  ```
  For tank-prototype-v2, the default sound playback & record device need to be changed by edit file /etc/pulse/default.pa, append following lines:
  ```bash
  set-default-sink alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo
  set-default-source alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo.monitor
  ```
  For other setups, please get available playback & record devices by following command:
  ```bash
  pactl list short sinks
  pactl list short sources
  ```
  Then edit /etc/pulse/default.pa according to your setup.

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

## Pairing PS4 joystick to Ubuntu Mate
  Pair with Ubuntu desktop tool "Bluetooth Manager".  
  To start pairing, hold "SHARE" and "PS" button in PS4 joystick until indicator flashes.  
  To test the joystick, run:
  ```bash
  jstest /dev/input/js0
  ```
