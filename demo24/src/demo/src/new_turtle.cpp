/*
    生成一只新的小乌龟
*/

#include "ros/ros.h"
#include "ros/service_client.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "new_turtle");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.创建一个乌龟的spawn请求信息
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle2";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 0.5;

    // 4.创建一个发送spawn信息的客户端
    ros::ServiceClient client = hd.serviceClient<turtlesim::Spawn>("/spawn");
    // 5.发送信息
    client.waitForExistence();
    bool flag = client.call(spawn);
    if (flag){
        ROS_INFO("Spawn Successed!");
    }else{
        ROS_ERROR("Error: Spawn Faild");
    }

    return 0;
}
