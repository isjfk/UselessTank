#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>

#include <thread>

#include "TankAgent.h"
#include "TankMsg.h"
#include "StdCrc32.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

char serialPort[2048];
char serialBaudrateStr[16];
speed_t serialBaudrate;

bool encoderInverted = false;
double encoderCountXPerMeter = 0;
double encoderDiffYawFullTurn = 0;

bool imuMsgPublishEnabled = true;
bool magMsgPublishEnabled = true;
bool odomImuMsgPublishEnabled = true;
bool odomEncoderMsgPublishEnabled = true;
bool odomMsgPublishEnabled = true;
bool odomTfBroadcastEnabled = true;

uint8_t logTankMsgConfig;
uint8_t logTankCmdConfig;

int serialFd = -1;
struct termios orgTty;

ros::NodeHandle *nodeHandle;

void loadParams(void);
void openSerialPort(void);
void closeSerialPort(void);


ros::Publisher *imuMsgPub;
ros::Publisher *magMsgPub;
ros::Publisher *odomMsgPub;
ros::Publisher *odomImuMsgPub;
ros::Publisher *odomEncoderMsgPub;
tf::TransformBroadcaster *tfBroadcaster;

uint32_t tankMsgCount = 0;

ros::Time currentTime;
uint32_t prevMsgTimestamp = 0;
int16_t prevEncoderLeft = 0;
int16_t prevEncoderRight = 0;
double imuTheta = 0;
double prevImuTheta = 0;
double imuOdomPose[3] = { 0, 0, 0 };
double imuOdomTwist[3] = { 0, 0, 0 };
double encoderOdomPose[3] = { 0, 0, 0 };
double encoderOdomTwist[3] = { 0, 0, 0 };
double odomPose[3] = { 0, 0, 0 };
double odomTwist[3] = { 0, 0, 0 };

void tankMsgThreadFunc(void);
error_t readTankMsg(TankMsg *tankMsg);
void tankMsgEncoderFix(TankMsg *tankMsg);
void publishImuMsg(TankMsg *tankMsg);
void publishMagMsg(TankMsg *tankMsg);
bool updateOdomData(TankMsg *tankMsg);
void publishOdomImuMsg(TankMsg *tankMsg);
void publishOdomEncoderMsg(TankMsg *tankMsg);
void publishOdomMsg(TankMsg *tankMsg);
void broadcastOdomTf(TankMsg *tankMsg);


void tankCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);


int main(int argc, char **argv) {
    ros::init(argc, argv, "tank_agent");

    ros::NodeHandle nh;
    nodeHandle = &nh;

    ROS_INFO("[TankAgent] Start TankAgent node.");

    loadParams();

    ros::Publisher ip = nh.advertise<sensor_msgs::Imu>("imu", 10);
    ros::Publisher mp = nh.advertise<sensor_msgs::MagneticField>("mag", 10);
    ros::Publisher op = nh.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher oip = nh.advertise<nav_msgs::Odometry>("odom_imu", 10);
    ros::Publisher oep = nh.advertise<nav_msgs::Odometry>("odom_encoder", 10);
    ros::Subscriber cmdVelSub = nh.subscribe("cmd_vel", 10, tankCmdCallback);
    tf::TransformBroadcaster tb;

    imuMsgPub = &ip;
    magMsgPub = &mp;
    odomMsgPub = &op;
    odomImuMsgPub = &oip;
    odomEncoderMsgPub = &oep;
    tfBroadcaster = &tb;

    openSerialPort();

    // Start TankMsg thread.
    std::thread tankMsgThread(tankMsgThreadFunc);

    ros::spin();

    tankMsgThread.join();
    closeSerialPort();
}


error_t readSize(void *buf, size_t size) {
    while (size > 0) {
        ssize_t len = read(serialFd, buf, size);
        if (len < 0) {
            ROS_WARN("[TankAgent] Read from serial error: %s", strerror(errno));
            return -1;
        }
        buf = ((uint8_t *) buf) + len;
        size -= len;
    }
    return 0;
}

error_t readTankMsgStartTag() {
    uint8_t tag = 0;
    size_t startTagCount = 0;
    size_t startTagCountMin = 5;
    size_t readCount = 0;

    while ((readSize(&tag, 1) == 0) && (readCount++ < sizeof(TankMsgPacket))) {
        if ((tag == 0xFF) && (startTagCount >= startTagCountMin)) {
            return 0;
        } else if (tag == 0x55) {
            startTagCount++;
        } else {
            startTagCount = 0;
        }
    }

    return -1;
}

error_t readTankMsg(TankMsg *tankMsg) {
    StdCrc32 crc32;

    memset(tankMsg, 0, sizeof(TankMsg));

    if (readTankMsgStartTag()) {
        return -1;
    }

    tankMsgCount++;

    if (readSize(tankMsgHeaderAddr(tankMsg), tankMsgHeaderSize(tankMsg))) {
        return -1;
    }

    stdCrc32Init(&crc32);
    stdCrc32Update(&crc32, tankMsgHeaderAddr(tankMsg), tankMsgHeaderCrcSize(tankMsg));

    uint32_t crcHeader = stdCrc32Get(&crc32);
    if (crcHeader != tankMsg->crcHeader) {
        ROS_WARN("[TankAgent] TankMsg[%d] Error validate TankMsg header CRC32 checksum, skip to next! Received CRC32[%08X] expected CRC32[%08X]", tankMsgCount, tankMsg->crcHeader, crcHeader);
        return -1;
    }
    if (tankMsg->desc.field.version != 1) {
        ROS_WARN("[TankAgent] TankMsg[%d] Unsupported TankMsg desc[%08X]!", tankMsgCount, tankMsg->desc.value);
        return -1;
    }
    if (tankMsg->dataLength > tankMsgDataSizeMax(tankMsg)) {
        ROS_WARN("[TankAgent] TankMsg[%d] Unsupported TankMsg dataLength[%d], max allowed[%d]!", tankMsgCount, tankMsg->dataLength, (uint32_t) tankMsgDataSizeMax(tankMsg));
        return -1;
    }

    if (readSize(tankMsgDataAddr(tankMsg), tankMsgDataSize(tankMsg))) {
        return -1;
    }

    stdCrc32Init(&crc32);
    stdCrc32Update(&crc32, tankMsgDataAddr(tankMsg), tankMsgDataCrcSize(tankMsg));

    uint32_t crcData = stdCrc32Get(&crc32);
    if (crcData != tankMsg->crcData) {
        ROS_WARN("[TankAgent] TankMsg[%d] Error validate TankMsg data CRC32 checksum, skip to next! Received CRC32[%08X] expected CRC32[%08X]", tankMsgCount, tankMsg->crcData, crcData);
        return -1;
    }

    return 0;
}

void logTankMsg(TankMsg *tankMsg) {
    if (logTankMsgConfig & 0b00000001) {
        uint8_t *buf = (uint8_t *) tankMsg;
        for (size_t i = 0; i < tankMsgSize(tankMsg); i++) {
            printf("%02X", (int) buf[i]);
        }
        printf("\r\n");
    }

    if (logTankMsgConfig & 0b00000010) {
        TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
        printf("TankMsg[%06d] seq[%06d] ts[%08d] gyro[%6.2f %6.2f %6.2f] accel[%6.2f %6.2f %6.2f] compass[%11.8f %11.8f %11.8f] quat[%6.2f %6.2f %6.2f %6.2f] encoder[%05u %05u]\r\n",
                tankMsgCount,
                tankMsg->seq,
                tankMsg->timestamp,
                data->gyro[0], data->gyro[1], data->gyro[2],
                data->accel[0], data->accel[1], data->accel[2],
                data->compass[0], data->compass[1], data->compass[2],
                data->quat[0], data->quat[1], data->quat[2], data->quat[3],
                data->motorEncoderLeft, data->motorEncoderRight);
    }
}

void tankMsgThreadFunc() {
    TankMsg _tankMsg;
    TankMsg *tankMsg = &_tankMsg;

    while (ros::ok()) {
        currentTime = ros::Time::now();

        if (readTankMsg(tankMsg)) {
            //ROS_WARN("[TankAgent] Read TankMsg from serial error!");
            continue;
        }

        tankMsgEncoderFix(tankMsg);
        logTankMsg(tankMsg);

        if (updateOdomData(tankMsg)) {
            publishImuMsg(tankMsg);
            publishMagMsg(tankMsg);
            publishOdomImuMsg(tankMsg);
            publishOdomEncoderMsg(tankMsg);
            publishOdomMsg(tankMsg);

            broadcastOdomTf(tankMsg);
        }

        TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
        prevMsgTimestamp = tankMsg->timestamp;
        prevEncoderLeft = (int16_t) data->motorEncoderLeft;
        prevEncoderRight = (int16_t) data->motorEncoderRight;
        prevImuTheta = imuTheta;
    }
}

void tankMsgEncoderFix(TankMsg *tankMsg) {
    if (!encoderInverted) {
        return;
    }

    TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
    uint16_t encoderLeft = (uint16_t) data->motorEncoderLeft;
    uint16_t encoderRight = (uint16_t) data->motorEncoderRight;

    encoderLeft = UINT16_MAX - encoderLeft + 1;
    encoderRight = UINT16_MAX - encoderRight + 1;

    data->motorEncoderLeft = encoderLeft;
    data->motorEncoderRight = encoderRight;
}

void publishImuMsg(TankMsg *tankMsg) {
    if (!imuMsgPublishEnabled) {
        return;
    }

    TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp = currentTime;
    imuMsg.header.frame_id = "imu_link";

    imuMsg.orientation.w = data->quat[0];
    imuMsg.orientation.x = data->quat[1];
    imuMsg.orientation.y = data->quat[2];
    imuMsg.orientation.z = data->quat[3];
    imuMsg.orientation_covariance[0] = 0.0025;
    imuMsg.orientation_covariance[1] = 0;
    imuMsg.orientation_covariance[2] = 0;
    imuMsg.orientation_covariance[3] = 0;
    imuMsg.orientation_covariance[4] = 0.0025;
    imuMsg.orientation_covariance[5] = 0;
    imuMsg.orientation_covariance[6] = 0;
    imuMsg.orientation_covariance[7] = 0;
    imuMsg.orientation_covariance[8] = 0.0025;

    imuMsg.angular_velocity.x = data->gyro[0];
    imuMsg.angular_velocity.y = data->gyro[1];
    imuMsg.angular_velocity.z = data->gyro[2];
    imuMsg.angular_velocity_covariance[0] = 0.02;
    imuMsg.angular_velocity_covariance[1] = 0;
    imuMsg.angular_velocity_covariance[2] = 0;
    imuMsg.angular_velocity_covariance[3] = 0;
    imuMsg.angular_velocity_covariance[4] = 0.02;
    imuMsg.angular_velocity_covariance[5] = 0;
    imuMsg.angular_velocity_covariance[6] = 0;
    imuMsg.angular_velocity_covariance[7] = 0;
    imuMsg.angular_velocity_covariance[8] = 0.02;

    imuMsg.linear_acceleration.x = data->accel[0];
    imuMsg.linear_acceleration.y = data->accel[1];
    imuMsg.linear_acceleration.z = data->accel[2];
    imuMsg.linear_acceleration_covariance[0] = 0.04;
    imuMsg.linear_acceleration_covariance[1] = 0;
    imuMsg.linear_acceleration_covariance[2] = 0;
    imuMsg.linear_acceleration_covariance[3] = 0;
    imuMsg.linear_acceleration_covariance[4] = 0.04;
    imuMsg.linear_acceleration_covariance[5] = 0;
    imuMsg.linear_acceleration_covariance[6] = 0;
    imuMsg.linear_acceleration_covariance[7] = 0;
    imuMsg.linear_acceleration_covariance[8] = 0.04;

    imuMsgPub->publish(imuMsg);
}

void publishMagMsg(TankMsg *tankMsg) {
    if (!magMsgPublishEnabled) {
        return;
    }

    TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
    sensor_msgs::MagneticField magMsg;
    magMsg.header.stamp = currentTime;
    magMsg.header.frame_id = "mag_link";

    magMsg.magnetic_field.x = data->compass[0];
    magMsg.magnetic_field.y = data->compass[1];
    magMsg.magnetic_field.z = data->compass[2];

    magMsgPub->publish(magMsg);
}

bool updateOdomData(TankMsg *tankMsg) {
    TankMsgSensorData *data = tankMsgDataPtrOfType(tankMsg, TankMsgSensorData);
    uint32_t timestamp = tankMsg->timestamp;
    if ((prevMsgTimestamp == 0) || (prevMsgTimestamp >= timestamp)) {
        // Tank control board reset, calculate imuTheta so it will not jump in next cycle
        float *quat = data->quat;
        imuTheta = atan2f(quat[1] * quat[2] + quat[0] * quat[3], 0.5f - quat[2] * quat[2] - quat[3] * quat[3]);
        return false;
    }

    double timeDiffMs = timestamp - prevMsgTimestamp;
    double timeDiff = timeDiffMs / 1000.0;

    int16_t encoderLeft = (int16_t) data->motorEncoderLeft;
    int16_t encoderRight = (int16_t) data->motorEncoderRight;
    int16_t encoderLeftDiff = encoderLeft - prevEncoderLeft;
    int16_t encoderRightDiff = encoderRight - prevEncoderRight;
    double encoderDiff = (encoderLeftDiff + encoderRightDiff) / 2;

    double encoderDistanceDiff = encoderDiff / encoderCountXPerMeter;
    double encoderSpeed = encoderDistanceDiff / timeDiff;
    double encoderThetaDiff = (-encoderLeftDiff + encoderRightDiff) / (encoderDiffYawFullTurn / 2) * M_PI;

    // Limit encoderTheta range in [-M_PI, +M_PI]
    double encoderTheta = encoderOdomPose[2] + encoderThetaDiff;
    if (encoderTheta > M_PI) {
        encoderTheta -= M_PI * 2;
    } else if (encoderTheta < (-M_PI)) {
        encoderTheta += M_PI * 2;
    }

    float *quat = data->quat;
    imuTheta = atan2f(quat[1] * quat[2] + quat[0] * quat[3], 0.5f - quat[2] * quat[2] - quat[3] * quat[3]);
    double imuThetaDiff = imuTheta - prevImuTheta;

    imuOdomPose[2] += imuThetaDiff;
    imuOdomTwist[2] = imuThetaDiff / timeDiff;

    encoderOdomPose[0] += encoderDistanceDiff * cos(encoderOdomPose[2] + (encoderThetaDiff / 2.0));
    encoderOdomPose[1] += encoderDistanceDiff * sin(encoderOdomPose[2] + (encoderThetaDiff / 2.0));
    encoderOdomPose[2] = encoderTheta;

    encoderOdomTwist[0] = encoderSpeed;
    encoderOdomTwist[1] = 0;
    encoderOdomTwist[2] = encoderThetaDiff / timeDiff;

    odomPose[0] += encoderDistanceDiff * cos(odomPose[2] + (imuThetaDiff / 2.0));
    odomPose[1] += encoderDistanceDiff * sin(odomPose[2] + (imuThetaDiff / 2.0));
    odomPose[2] += imuThetaDiff;

    odomTwist[0] = encoderSpeed;
    odomTwist[1] = 0;
    odomTwist[2] = imuThetaDiff / timeDiff;

    return true;
}

void publishOdomImuMsg(TankMsg *tankMsg) {
    if (!odomImuMsgPublishEnabled) {
        return;
    }

    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = 0;
    odomMsg.pose.pose.position.y = 0;
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(imuOdomPose[2]);

    odomMsg.twist.twist.linear.x = 0;
    odomMsg.twist.twist.linear.y = 0;
    odomMsg.twist.twist.linear.z = 0;

    odomMsg.twist.twist.angular.x = 0;
    odomMsg.twist.twist.angular.y = 0;
    odomMsg.twist.twist.angular.z = imuOdomTwist[2];

    odomImuMsgPub->publish(odomMsg);
}

void publishOdomEncoderMsg(TankMsg *tankMsg) {
    if (!odomEncoderMsgPublishEnabled) {
        return;
    }

    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = encoderOdomPose[0];
    odomMsg.pose.pose.position.y = encoderOdomPose[1];
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(encoderOdomPose[2]);

    odomMsg.twist.twist.linear.x = encoderOdomTwist[0];
    odomMsg.twist.twist.linear.y = 0;
    odomMsg.twist.twist.linear.z = 0;

    odomMsg.twist.twist.angular.x = 0;
    odomMsg.twist.twist.angular.y = 0;
    odomMsg.twist.twist.angular.z = encoderOdomTwist[2];

    odomEncoderMsgPub->publish(odomMsg);
}

void publishOdomMsg(TankMsg *tankMsg) {
    if (!odomMsgPublishEnabled) {
        return;
    }

    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = odomPose[0];
    odomMsg.pose.pose.position.y = odomPose[1];
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomPose[2]);

    odomMsg.twist.twist.linear.x = odomTwist[0];
    odomMsg.twist.twist.linear.y = 0;
    odomMsg.twist.twist.linear.z = 0;

    odomMsg.twist.twist.angular.x = 0;
    odomMsg.twist.twist.angular.y = 0;
    odomMsg.twist.twist.angular.z = odomTwist[2];

    odomMsgPub->publish(odomMsg);
}

void broadcastOdomTf(TankMsg *tankMsg) {
    if (!odomTfBroadcastEnabled) {
        return;
    }

    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = currentTime;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base_link";

    odomTrans.transform.translation.x = odomPose[0];
    odomTrans.transform.translation.y = odomPose[1];
    odomTrans.transform.translation.z = 0;
    odomTrans.transform.rotation = tf::createQuaternionMsgFromYaw(odomPose[2]);

    tfBroadcaster->sendTransform(odomTrans);
}


error_t writeSize(void *buf, size_t size) {
    while (size > 0) {
        ssize_t len = write(serialFd, buf, size);
        if (len < 0) {
            ROS_WARN("[TankAgent] Write to serial error: %s", strerror(errno));
            return -1;
        }
        buf = ((uint8_t *) buf) + len;
        size -= len;
    }
    return 0;
}

void tankCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    const double speedMax = 0.5;        // Unit: meter/second.
    const double turnMax = M_PI;        // Unit: rad/second. To degree/second: turnMax / (2 * M_PI) * 360.0
    const int tankRcMax = 30000;
    double speed = cmd_vel->linear.x;
    double turn = cmd_vel->angular.z;

    speed = (speed < -speedMax) ? -speedMax : speed;
    speed = (speed > speedMax) ? speedMax : speed;
    turn = (turn < -turnMax) ? -turnMax : turn;
    turn = (turn > turnMax) ? turnMax : turn;

    int tankThrottle = (int) (speed / speedMax * tankRcMax);
    int tankYaw = (int) (turn / turnMax * tankRcMax);

    char buf[1024];
    int size = sprintf(buf, "$AP0:%dX%dY!", tankYaw, tankThrottle);

    writeSize(buf, size);

    if (logTankCmdConfig & 0x00000001) {
        printf("Write TankCmd[%s] to serial.\r\n", buf);
    }
}


void loadParams(void) {
    std::string portStr;
    ros::param::param<std::string>("~serialPort", portStr, "/dev/ttyS0");
    strncpy(serialPort, portStr.c_str(), sizeof(serialPort));

    std::string baudStr;
    ros::param::param<std::string>("~serialBaudrate", baudStr, "115200");
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
        ROS_ERROR("[TankAgent] Node Exit - Unsupported serial baudrate[%s]!", baudStr.c_str());
        exit(-1);
    }

    ros::param::param<bool>("~encoderInverted", encoderInverted, false);
    ros::param::param<double>("~encoderCountXPerMeter", encoderCountXPerMeter, 0);
    ros::param::param<double>("~encoderDiffYawFullTurn", encoderDiffYawFullTurn, 0);

    if (encoderCountXPerMeter == 0) {
        ROS_ERROR("[TankAgent] Node Exit - Incorrect encoderCountXPerMeter[%f]! Check your env: TANK_MOTOR_MODEL", encoderCountXPerMeter);
        exit(-1);
    }

    if (encoderDiffYawFullTurn == 0) {
        ROS_ERROR("[TankAgent] Node Exit - Incorrect encoderDiffYawFullTurn[%f]! Check your env: TANK_MOTOR_MODEL", encoderDiffYawFullTurn);
        exit(-1);
    }

    ros::param::param<bool>("~imuMsgPublishEnabled", imuMsgPublishEnabled, true);
    ros::param::param<bool>("~magMsgPublishEnabled", magMsgPublishEnabled, true);
    ros::param::param<bool>("~odomImuMsgPublishEnabled", odomImuMsgPublishEnabled, true);
    ros::param::param<bool>("~odomEncoderMsgPublishEnabled", odomEncoderMsgPublishEnabled, true);
    ros::param::param<bool>("~odomMsgPublishEnabled", odomMsgPublishEnabled, true);
    ros::param::param<bool>("~odomTfBroadcastEnabled", odomTfBroadcastEnabled, true);

    std::string logTankMsgStr;
    ros::param::param<std::string>("~logTankMsg", logTankMsgStr, "0");
    logTankMsgConfig = strtoul(logTankMsgStr.c_str(), NULL, 2);

    std::string logTankCmdStr;
    ros::param::param<std::string>("~logTankCmd", logTankCmdStr, "0");
    logTankCmdConfig = strtoul(logTankCmdStr.c_str(), NULL, 2);
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
    tty.c_cc[VTIME] = (int) (3 * 10);   // Read timeout (0 disable read timeout): 0 seconds.
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
