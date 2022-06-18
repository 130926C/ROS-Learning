#include "ros/ros.h"
#include "turtlesim/Spawn.h"


int main(int argc, char *argv[]){
    ROS_INFO("Create a New turrle [CPP]");
    // 1.初始化节点
    ros::init(argc, argv, "NT_node");
    // 2.申请句柄
    ros::NodeHandle hd;
    // 3.创造客户端
    ros::ServiceClient client = hd.serviceClient<turtlesim::Spawn>("/spawn");
    // 4.发送请求
    turtlesim::Spawn add_new;
    const char* nameList[] = {"zhang3", "li4", "wang5", "zhao6"};

    add_new.request.name;
    add_new.request.x = 2;
    add_new.request.y = 3;

    for (auto *name : nameList){
        add_new.request.name = name;
        add_new.request.x += 3;
        add_new.request.y += 2;
        client.waitForExistence();
        client.call(add_new);
        ROS_INFO("Turtle %s crated at [%.2f,%.2f]", name, add_new.request.x, add_new.request.y);
    }

    return 0;
}
