#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tank_setup_tf_node");

    ros::NodeHandle nh;
    ros::Rate rate(100);
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion quat;
    tf::Vector3 vect(0.12, 0.0, 0.16);
    //tf::Vector3 vect(0.08, 0.0, 0.20);

    ROS_INFO("[TankSetupTf] Start Tank TF node.");

    // Rotate laser data 180 degree to correct laser angle.
    quat.setRPY(0, 0, M_PI);

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

