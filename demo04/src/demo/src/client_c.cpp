#include "ros/ros.h"
#include "demo/AddNum.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is Client [CPP-demo04]");
    if (argc == 1){
        ROS_INFO("NO number input, use default [10, 20]");
    }else if (argc == 3){
        ROS_INFO("Detect TWO input [%s, %s]", argv[1], argv[2]);
    }else {
        ROS_INFO("Error, Input number must TWO");
        return 0;
    }

    // 1.初始化节点
    ros::init(argc, argv, "client_node");
    // 2.创造句柄
    ros::NodeHandle hd;
    // 3.创造客户端 & 关注话题
    ros::ServiceClient client = hd.serviceClient<demo::AddNum>("addNum");
    // 4,发送请求

    demo::AddNum req;
    if (argc == 1){
        req.request.num1 = 10;
        req.request.num2 = 20;
    }else {
        req.request.num1 = atoi(argv[1]);
        req.request.num2 = atoi(argv[2]);
    }
    client.waitForExistence();

    ROS_INFO("Send Request [%d, %d]", req.request.num1, req.request.num2);
    bool flag = client.call(req);
    if (flag){
        ROS_INFO("Server success! response: %d as request [%d,%d]", req.response.sum, req.request.num1, req.request.num2);
    }else {
        ROS_INFO("Error, Server reject request");
        return 0;
    }

    return 0;
}
