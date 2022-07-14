#include "ros/ros.h"
#include "turtlesim/Spawn.h"

// 下面是B站视频上的代码部分，使用的是Client方式创建服务
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

// 其实ROS提供了直接call server的方式，如下
//      使用ros::service::call这种方式就可以少创建一个client对象，对于没有什么交互的需求而言是比较合适的
int main_(int argc, char* argv[]){
    ros::init(argc, argv, "NT_node");
    ros::NodeHandle hd;

    turtlesim::Spawn spawn;
    spawn.request.name = "zhang3";
    spawn.request.theta = 0.0;
    spawn.request.x = 2.0;
    spawn.request.y = 3.0;

    // 【注意】这里有一个坑点，需要先等 /spawn 服务被拉起来后再去call它，要不然在窗口上是不显示的。
    ros::service::waitForService("/spawn");
    ros::service::call("/spawn",spawn);

    return 0;
}