#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int tfRate = 20;

double laserTfX = 0;
double laserTfY = 0;
double laserTfZ = 0;
double laserTfRoll = 0;
double laserTfPitch = 0;
double laserTfYaw = 0;

bool depthTfEnabled = false;
double depthTfX = 0;
double depthTfY = 0;
double depthTfZ = 0;
double depthTfRoll = 0;
double depthTfPitch = 0;
double depthTfYaw = 0;

void loadParams(void);

int main(int argc, char** argv) {
    ros::init(argc, argv, "tank_setup_tf_node");
    ros::NodeHandle nh;

    loadParams();

    tf::TransformBroadcaster broadcaster;

    ros::Rate rate(tfRate);

    tf::Vector3 laserVect(laserTfX, laserTfY, laserTfZ);
    tf::Quaternion laserQuat;
    laserQuat.setRPY(laserTfRoll, laserTfPitch, laserTfYaw);

    tf::Vector3 depthVect(depthTfX, depthTfY, depthTfZ);
    tf::Quaternion depthQuat;
    depthQuat.setRPY(depthTfRoll, depthTfPitch, depthTfYaw);

    tf::Vector3 imuVect(0, 0, 0);
    tf::Quaternion imuQuat;
    imuQuat.setRPY(0, 0, 0);

    ROS_INFO("[TankSetupTf] Start Tank Transform node: rate[%d].", tfRate);
    ROS_INFO("[TankSetupTf] Laser rader Transform: x[%f] y[%f] z[%f] roll[%f] pitch[%f] yaw[%f].", laserTfX, laserTfY, laserTfZ, laserTfRoll, laserTfPitch, laserTfYaw);

    while (nh.ok()) {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(laserQuat, laserVect),
                ros::Time::now(),
                "base_link",
                "base_laser"));

        if (depthTfEnabled) {
            broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(depthQuat, depthVect),
                    ros::Time::now(),
                    "base_link",
                    "base_depth"));
        }

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(imuQuat, imuVect),
                ros::Time::now(),
                "base_link",
                "imu_link"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(imuQuat, imuVect),
                ros::Time::now(),
                "base_link",
                "mag_link"));

        rate.sleep();
    }
}

void loadParams(void) {
    ros::param::param<int>("~tfRate", tfRate, 0);

    if (tfRate == 0) {
        ROS_ERROR("[TankSetupTf] Node Exit - Incorrect tfRate[%d]! Check your env: TANK_MODEL", tfRate);
        exit(-1);
    }

    ros::param::param<double>("~laserTfX", laserTfX, 0);
    ros::param::param<double>("~laserTfY", laserTfY, 0);
    ros::param::param<double>("~laserTfZ", laserTfZ, 0);
    ros::param::param<double>("~laserTfRoll", laserTfRoll, 0);
    ros::param::param<double>("~laserTfPitch", laserTfPitch, 0);
    ros::param::param<double>("~laserTfYaw", laserTfYaw, 0);

    ros::param::param<bool>("~depthTfEnabled", depthTfEnabled, false);
    ros::param::param<double>("~depthTfX", depthTfX, 0);
    ros::param::param<double>("~depthTfY", depthTfY, 0);
    ros::param::param<double>("~depthTfZ", depthTfZ, 0);
    ros::param::param<double>("~depthTfRoll", depthTfRoll, 0);
    ros::param::param<double>("~depthTfPitch", depthTfPitch, 0);
    ros::param::param<double>("~depthTfYaw", depthTfYaw, 0);
}
