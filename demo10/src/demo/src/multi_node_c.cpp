#include "ros/ros.h"

int main(int argc, char *argv[]){
    ROS_INFO("Node multi start");
    // 1.初始化节点
    ros::init(argc, argv, "multi_node", ros::init_options::AnonymousName);
    // 2.创建句柄
    ros::NodeHandle hd;

    // 3.循环做一些无意义的事
    int curIndex = 0;
    ros::Rate rate(1);
    while(ros::ok()){
        ROS_INFO("mulit_node working current Index=%d", curIndex);
        curIndex++;
        rate.sleep();
    }


    return 0;
}
