#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "Q2E_node");
    ros::NodeHandle hd;

    nav_msgs::Odometry position;
    tf::Quaternion RQ2;

    double roll,pitch,yaw;

    position.pose.pose.orientation.w = 0.012;
    position.pose.pose.orientation.x = 0.076;
    position.pose.pose.orientation.y = 0.012;
    position.pose.pose.orientation.z = 0.098;

    ROS_INFO("Quaternion angle [%.2f, %.2f, %.2f, %.2f]",
        position.pose.pose.orientation.w,
        position.pose.pose.orientation.x,
        position.pose.pose.orientation.y,
        position.pose.pose.orientation.z    
    );

    tf::quaternionMsgToTF(position.pose.pose.orientation, RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);

    ROS_INFO("Euler angle [%.2f, %.2f, %.2f]", roll, pitch, yaw);

    return 0;
}
