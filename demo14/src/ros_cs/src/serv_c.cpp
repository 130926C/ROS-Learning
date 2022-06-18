#include "ros/ros.h"
#include "ros_cs/serv_msg.h"

bool dealRequest(
    ros_cs::serv_msgRequest &req, 
    ros_cs::serv_msgResponse &res
){
    ROS_INFO("Receive Request is: [num1=%d, num2=%d]", req.num1, req.num2);
    res.sum = req.num1 + req.num2;
    ROS_INFO("Send Response is: [sum=%d]", res.sum);
    ROS_INFO("========================================");
    return true;
}


int main(int argc, char *argv[]){
    ROS_INFO("This is Server node");
    // 1.初始化节点
    ros::init(argc, argv, "server_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建服务 & 注册的话题
    ros::ServiceServer server = hd.advertiseService("server_topic", dealRequest);

    ros::spin();

    return 0;
}
