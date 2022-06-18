#include "ros/ros.h"


int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "log_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.输出日志

    ros::Rate rate(0.3);
    int count = 0;

    while(ros::ok()){
        ROS_INFO("Count:%d message", count++);
        ROS_DEBUG("ROS DEBUG MESSAGE");
        ROS_INFO("ROS INFO MESSAGE");
        ROS_WARN("ROS WARNING MESSAGE");
        ROS_ERROR("ROS ERROR MESSAGE");
        ROS_FATAL("ROS FATAL MESSAGE");
        ROS_INFO("-------------------------");
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
