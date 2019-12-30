#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int tfRate = 10;

double laserTfX = 0;
double laserTfY = 0;
double laserTfZ = 0;
double laserTfRoll = 0;
double laserTfPitch = 0;
double laserTfYaw = 0;

void loadParams(void);

int main(int argc, char** argv) {
    ros::init(argc, argv, "tank_setup_tf_node");
    ros::NodeHandle nh;

    loadParams();

    tf::TransformBroadcaster broadcaster;

    ros::Rate rate(tfRate);
    tf::Vector3 vect(laserTfX, laserTfY, laserTfZ);
    tf::Quaternion quat;
    quat.setRPY(laserTfRoll, laserTfPitch, laserTfYaw);

    ROS_INFO("[TankSetupTf] Start Tank Transform node: rate[%d].", tfRate);
    ROS_INFO("[TankSetupTf] Laser rader Transform: x[%f] y[%f] z[%f] roll[%f] pitch[%f] yaw[%f].", laserTfX, laserTfY, laserTfZ, laserTfRoll, laserTfPitch, laserTfYaw);

    while (nh.ok()) {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(quat, vect),
                ros::Time::now(),
                "base_link",
                "base_laser"));

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
}
