#include "ros/ros.h"
#include "ros_cs/serv_msg.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is client node");
    // 1.初始化节点
    ros::init(argc, argv, "client_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建客户端
    ros::ServiceClient client = hd.serviceClient<ros_cs::serv_msg>("server_topic");

    // 4.发送请求
    ros_cs::serv_msg req;
    req.request.num1 = 10;
    req.request.num2 = 13;

    client.waitForExistence();
    ROS_INFO("Request [num1=%d, num2=%d] send", req.request.num1, req.request.num2);
    client.call(req);
    
    return 0;
}
