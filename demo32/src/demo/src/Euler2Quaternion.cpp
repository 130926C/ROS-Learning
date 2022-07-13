#include "ros/ros.h"
#include "tf/tf.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "Q2E_node");
    ros::NodeHandle hd;

    geometry_msgs::Quaternion q;
    double roll=0.5,pitch=4.3,yaw=5.3;

    ROS_INFO("Euler angle [%.2f, %.2f, %.2f]", roll, pitch, yaw);
    q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    ROS_INFO("Quaternion angle [%.2f, %.2f, %.2f, %.2f]", q.w, q.x, q.y, q.z);

    return 0;
}
