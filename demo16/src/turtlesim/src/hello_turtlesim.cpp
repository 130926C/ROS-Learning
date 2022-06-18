#include "ros/ros.h"

int main(int argc, char  *argv[]){
    ROS_INFO("This is self-made turtlesim hello_turtlesim.cpp");
    // 1.初始化节点
    ros::init(argc, argv, "hello_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.输出
    ros::Rate rate(1);
    while(ros::ok()){
        ROS_INFO("Hello from self made turtlesim package");
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
