#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>

#include <thread>

#include "TankAgent.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"

char serialPort[2048];
char serialBaudrateStr[16];
speed_t serialBaudrate;

int serialFd = -1;
struct termios orgTty;

ros::NodeHandle *nodeHandle;
ros::Publisher *imuMsgPub;
ros::Publisher *magMsgPub;
ros::Publisher *odomMsgPub;

void parseSerialParam();
void openSerialPort();
void closeSerialPort();

void tankMsgThreadFunc();
error_t readTankMsg(TankMsg *tankMsg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "tank_agent");

    ros::NodeHandle nh;
    ros::Publisher ip = nh.advertise<sensor_msgs::Imu>("/tank/imu", 10);
    ros::Publisher mp = nh.advertise<sensor_msgs::MagneticField>("/tank/mag", 10);
    ros::Publisher op = nh.advertise<nav_msgs::Odometry>("/tank/odom", 10);

    nodeHandle = &nh;
    imuMsgPub = &ip;
    magMsgPub = &mp;
    odomMsgPub = &op;

    parseSerialParam();
    openSerialPort();

    std::thread tankMsgThread(tankMsgThreadFunc);

    while (ros::ok()) {
        ros::spinOnce();
    }

    tankMsgThread.join();
    closeSerialPort();
}

error_t readSize(void *buf, size_t size) {
    while (size > 0) {
        ssize_t readLen = read(serialFd, buf, size);
        if (readLen < 0) {
            ROS_WARN("[TankAgent] Read from serial error: %s", strerror(errno));
            return -1;
        }
        buf = ((uint8_t *) buf) + readLen;
        size -= readLen;
    }
    return 0;
}

int isStartTag(uint8_t *buf) {
    return (buf[0] == 0x55) && (buf[1] == 0xAA);
}

error_t readTankMsgHeader(uint32_t *header) {
    uint8_t buf[4];

    memset(buf, 0, sizeof(buf));

    while (!isStartTag(buf)) {
        if (readSize(buf, 2)) {
            return -1;
        }
        if (buf[1] == 0x55) {
            buf[0] = buf[1];
            if (readSize(buf + 1, 1)) {
                return -1;
            }
        }
    }

    while (isStartTag(buf)) {
        if (readSize(buf, 2)) {
            return -1;
        }
    }

    if (readSize(buf + 2, 2)) {
        return -1;
    }

    *header = *((uint32_t *) buf);

    return 0;
}

error_t readTankMsg(TankMsg *tankMsg) {
    if (readTankMsgHeader(&tankMsg->header)) {
        return -1;
    }
    tankMsg->startTag = 0xAA55AA55AA55AA55;
    if (tankMsg->header != 0x00000001) {
        ROS_WARN("[TankAgent] Unsupported TankMsg header[%08X]!", tankMsg->header);
        return -1;
    }

    if (readSize(&tankMsg->dataLength, sizeof(tankMsg->dataLength))) {
        return -1;
    }
    if (tankMsg->dataLength > 512) {
        ROS_WARN("[TankAgent] Unsupported TankMsg dataLength[%d]!", tankMsg->dataLength);
        return -1;
    }

    if (readSize(&tankMsg->data, tankMsg->dataLength)) {
        return -1;
    }

    if (readSize(&tankMsg->crc, sizeof(tankMsg->crc))) {
        return -1;
    }

//    uint8_t *buf = (uint8_t *) tankMsg;
//    for (size_t i = 0; i < sizeof(TankMsg); i++) {
//        printf("%02X", (int) buf[i]);
//    }
//    printf("\r\n");
    printf("gyro[%8.2f %8.2f %8.2f] accel[%8.2f %8.2f %8.2f] compass[%8.2f %8.2f %8.2f] quat[%8.2f %8.2f %8.2f %8.2f]\r\n",
            tankMsg->data.gyro[0], tankMsg->data.gyro[1], tankMsg->data.gyro[2],
            tankMsg->data.accel[0], tankMsg->data.accel[1], tankMsg->data.accel[2],
            tankMsg->data.compass[0], tankMsg->data.compass[1], tankMsg->data.compass[2],
            tankMsg->data.quat[0], tankMsg->data.quat[1], tankMsg->data.quat[2], tankMsg->data.quat[3]);

    return 0;
}

void tankMsgThreadFunc() {
    TankMsg tankMsg;

    while (ros::ok()) {
        ros::Time currentTime = ros::Time::now();

        if (readTankMsg(&tankMsg)) {
            ROS_WARN("[TankAgent] Read TankMsg from serial error!");
            continue;
        }

        sensor_msgs::Imu imuMsg;
        imuMsg.header.stamp = currentTime;
        imuMsg.header.frame_id = "imu_link";

        imuMsg.orientation.w = tankMsg.data.quat[0];
        imuMsg.orientation.x = tankMsg.data.quat[1];
        imuMsg.orientation.y = tankMsg.data.quat[2];
        imuMsg.orientation.z = tankMsg.data.quat[3];

        imuMsg.angular_velocity.x = tankMsg.data.gyro[0];
        imuMsg.angular_velocity.y = tankMsg.data.gyro[1];
        imuMsg.angular_velocity.z = tankMsg.data.gyro[2];

        imuMsg.linear_acceleration.x = tankMsg.data.accel[0];
        imuMsg.linear_acceleration.y = tankMsg.data.accel[1];
        imuMsg.linear_acceleration.z = tankMsg.data.accel[2];

        imuMsgPub->publish(imuMsg);


        sensor_msgs::MagneticField magMsg;
        magMsg.header.stamp = currentTime;
        magMsg.header.frame_id = "mag_link";

        magMsg.magnetic_field.x = tankMsg.data.compass[0];
        magMsg.magnetic_field.y = tankMsg.data.compass[1];
        magMsg.magnetic_field.z = tankMsg.data.compass[2];

        magMsgPub->publish(magMsg);


        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = currentTime;
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_link";

        odomMsg.pose.pose.position.x = 311;
        odomMsg.pose.pose.position.y = 312;
        odomMsg.pose.pose.position.z = 313;

        odomMsg.twist.twist.linear.x = 321;
        odomMsg.twist.twist.linear.y = 322;
        odomMsg.twist.twist.linear.z = 323;

        odomMsg.twist.twist.angular.x = 331;
        odomMsg.twist.twist.angular.y = 332;
        odomMsg.twist.twist.angular.z = 333;

        odomMsgPub->publish(odomMsg);
    }
}

void parseSerialParam() {
    std::string portStr;
    std::string baudStr;

    nodeHandle->param<std::string>("serialPort", portStr, "/dev/ttyS0");
    strncpy(serialPort, portStr.c_str(), sizeof(serialPort));

    nodeHandle->param<std::string>("serialBaudrate", baudStr, "115200");
    strncpy(serialBaudrateStr, baudStr.c_str(), sizeof(serialBaudrateStr));

    if (baudStr == "9600") {
        serialBaudrate = B9600;
    } else if (baudStr == "19200") {
        serialBaudrate = B19200;
    } else if (baudStr == "38400") {
        serialBaudrate = B38400;
    } else if (baudStr == "57600") {
        serialBaudrate = B57600;
    } else if (baudStr == "115200") {
        serialBaudrate = B115200;
#ifdef B230400
    } else if (baudStr == "230400") {
        serialBaudrate = B230400;
#endif
#ifdef B460800
    } else if (baudStr == "460800") {
        serialBaudrate = B460800;
#endif
#ifdef B500000
    } else if (baudStr == "500000") {
        serialBaudrate = B500000;
#endif
#ifdef B576000
    } else if (baudStr == "576000") {
        serialBaudrate = B576000;
#endif
#ifdef B921600
    } else if (baudStr == "921600") {
        serialBaudrate = B921600;
#endif
#ifdef B1000000
    } else if (baudStr == "1000000") {
        serialBaudrate = B1000000;
#endif
#ifdef B1152000
    } else if (baudStr == "1152000") {
        serialBaudrate = B1152000;
#endif
#ifdef B1500000
    } else if (baudStr == "1500000") {
        serialBaudrate = B1500000;
#endif
#ifdef B2000000
    } else if (baudStr == "2000000") {
        serialBaudrate = B2000000;
#endif
#ifdef B2500000
    } else if (baudStr == "2500000") {
        serialBaudrate = B2500000;
#endif
#ifdef B3000000
    } else if (baudStr == "3000000") {
        serialBaudrate = B3000000;
#endif
#ifdef B3500000
    } else if (baudStr == "3500000") {
        serialBaudrate = B3500000;
#endif
#ifdef B4000000
    } else if (baudStr == "4000000") {
        serialBaudrate = B4000000;
#endif
    } else {
        ROS_ERROR("[TankAgent] Node Exit - unsupported serial baudrate[%s]!", baudStr.c_str());
        exit(-1);
    }
}

void openSerialPort() {
    serialFd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        ROS_ERROR("[TankAgent] Node Exit - Error open serial port[%s]: %s", serialPort, strerror(errno));
        exit(-1);
    } else {
        ROS_INFO("[TankAgent] Open serial port[%s] baud[%s] success.", serialPort, serialBaudrateStr);
    }
    fcntl(serialFd, F_SETFL, 0);

    // Save current serial port setting.
    tcgetattr(serialFd, &orgTty);

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(serialFd, &tty);

    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;                    // Non-canonical mode.
    tty.c_lflag = 0;                    // Non-canonical mode.
    tty.c_cc[VTIME] = (int) (0 * 10);   // Read timeout (0 disable read timeout): 0 seconds.
    tty.c_cc[VMIN] = 1;                 // Blocking read until at least 1 byte received.

    cfsetispeed(&tty, serialBaudrate);
    cfsetospeed(&tty, serialBaudrate);

    tcflush(serialFd, TCIFLUSH);
    tcsetattr(serialFd, TCSANOW, &tty);
}

void closeSerialPort() {
    // Restore serial port setting.
    tcsetattr(serialFd, TCSANOW, &orgTty);

    close(serialFd);
}

