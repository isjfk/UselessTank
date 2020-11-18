# Install Ubuntu Desktop 20.04 LTS amd64

## Prepare the USB driver with Ubuntu image
Flash image "ubuntu-20.04-desktop-amd64.iso" into USB-disk by balenaEtcher.  
Boot computer with the USB driver and install Ubuntu, change following options while install:
- Installtion Type
  Erase disk and install Ubuntu
- Where are you?
  China Time
- Who are you?
  - Your name
    tank
  - Your computer's name
    tank-xxx  
    Replace xxx with number start from 101. E.g. tank-101.
  - Pick a username
    tank
  - Password
    Initial0
  - Select "Log in automatically"

## Configure WiFi
  Login into Ubuntu desktop, connect to WiFi in status bar of upper right conor.  
  Select "Protected EAP (PEAP)" in Authentication.  
  Select "No CA certificate is required".  
  Input you domain account in Username & Password.  
  Click "Connect".  

## Change Ubuntu settings
  In Ubuntu open "Software & Updates". Change:
  - Download from
    To "Server for United States".
  In Ubuntu open "Settings", Change:
  - Power -> Blank Screen
    To "10 minutes".
  - Privacy -> Screen Lock
    Automatic Screen Lock: off  
    Lock Screen on Suspend: off

## Update Ubuntu to latest
  In Ubuntu open bash shell (Ctrl+Alt+T in desktop):
  ```bash
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get dist-upgrade
  ```

## Install basic software packages & enable SSH remote login
  In Ubuntu open bash shell (Ctrl+Alt+T in desktop):
  ```bash
  sudo apt-get install net-tools openssh-server
  ```
  In Ubuntu open "Ubuntu Software", install:
  - Chromium

## Copy ssh private key
  Copy ssh private key files into ~/.ssh to enable ssh login by certification.

# Install ROS

## Install ROS automatically by script
  Transfer directory ./init-setup to Ubuntu ~/ directory by SFTP.  
  Login into Ubuntu bash shell.
  ```bash
  cd ~/init-setup
  chmod a+x *.sh
  sudo ./init-setup.sh
  ```
  After script finshed in success, it will reboot automatically.  
  If you got any errors occur during execute init-setup.sh, please execute rest of the steps manually. Otherwise there may redundance lines writen into some configuration files.

## Install ROS manually
  If you execute init-setup.sh already then you should omit this section.

  In case you prefer the manual way, executes scripts in "__init_scripts.sh" one by one. Keep in mind most of the commands you need to execute by sudo.

## Setup tank model
  Edit file ~/tank_model.env, uncomment lines that match the tank setup.

# Device Configuration

## Setup depth camera

### Install librealsense for depth camera
  ```bash
  sudo apt-get install libglfw3-dev
  cd ~/packages
  git clone https://github.com/IntelRealSense/librealsense.git
  cd librealsense
  ./scripts/setup_udev_rules.sh
  mkdir build && cd build
  cmake ../ -DCMAKE_BUILD_TYPE=Release
  sudo make uninstall
  make clean && make
  sudo make install
  ```

### Upgrade firmware for depth camera
  ```bash
  cd ~/packages/librealsense/build/common/fw
  rs-fw-update -l
  rs-fw-update -s <sn> -f ~/D4XX_FW_Image-<version>.bin
  ```

## Pairing XBox One joystick to Ubuntu
  In Ubuntu open "Settings -> Bluetooth".  
  To start pairing, hold "SHARE" and "PS" button in PS4 joystick until indicator flashes.  
  To test the joystick, run:
  ```bash
  jstest /dev/input/js0
  ```
