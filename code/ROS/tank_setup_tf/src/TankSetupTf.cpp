#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tank_setup_tf");

    ros::NodeHandle nh;
    ros::Rate rate(100);
    tf::TransformBroadcaster broadcaster;

    ROS_INFO("[TankSetupTf] Start Tank TF node.");

    while (nh.ok()) {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.07, 0.0, 0.09)),
                ros::Time::now(),
                "base_link",
                "base_laser"));

        rate.sleep();
    }
}

