#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "time.h"

void dealMsg(const turtlesim::Pose::ConstPtr &pose){
    ROS_INFO("x=%.3f\ny=%.3f\ntheta=%.3f\nangular_velocity=%.3f\nlinear_velocity=%.3f", pose->x, pose->y, pose->theta, pose->angular_velocity, pose->linear_velocity);
    sleep(1);
}

int main(int argc, char *argv[]){
    ROS_INFO("This is subscriber [CPP]");
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2,创建句柄
    ros::NodeHandle hd;
    // 3.创建订阅者 & 关注话题
    ros::Subscriber sub = hd.subscribe("/turtle1/pose", 10, dealMsg);
    ros::spin();
    
    return 0;
}
