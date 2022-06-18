#include "ros/ros.h"

int main(int argc, char *argv[]){
    ROS_INFO("ROS shutdown node");
    // 1.初始化节点
    ros::init(argc, argv, "shutdown_node");
    // 2.创建节点句柄
    ros::NodeHandle hd;

    // 3.模拟工作
    ROS_INFO("Simulat working...");
    ros::Duration(2);
    ROS_INFO("Working finished");
    ros::shutdown();

    ROS_INFO("After shutdown code");

    
    return 0;
}
