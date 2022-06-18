#include "ros/ros.h"
#include "demo/hello.h"

int main(int argc, char *argv[]){
    ROS_INFO("Hello demo");

    // 1.初始化节点
    ros::init(argc, argv, "hello_node");

    // 2.运行
    hello_ns::MyHello myHello;
    myHello.run();

    return 0;
}
