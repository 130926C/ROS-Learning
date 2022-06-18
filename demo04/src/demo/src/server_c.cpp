#include "ros/ros.h"
#include "demo/AddNum.h"

bool dealRequest(
    demo::AddNumRequest &req,
    demo::AddNumResponse &rep
){
    int num1 = req.num1;
    int num2 = req.num2;
    ROS_INFO("Reveived Request: num_1=%d, num_2=%d, Response is sum=%d", num1, num2, num1+num2);
    rep.sum = num1 + num2;
    return true;
}


int main(int argc, char *argv[]){
    ROS_INFO("This is server [CPP-demo04]");
    // 1.初始化节点
    ros::init(argc, argv, "server_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建服务 & 注册话题
    ros::ServiceServer server = hd.advertiseService("addNum", dealRequest);
    // 4.spin
    ros::spin();

    return 0;
}
