#include "ros/ros.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is log message");
    // 1.初始化节点
    ros::init(argc, argv, "log_node");
    // 2.创建节点句柄
    ros::NodeHandle hd;

    // 3.输出信息
    ROS_INFO("ROS INFO Message");
    ROS_DEBUG("ROS DEBUG Message");
    ROS_WARN("ROS WARN Message");
    ROS_ERROR("ROS ERROR Message");
    ROS_FATAL("ROS FATAL Message");

    ROS_INFO("after message info");

    return 0;
}
