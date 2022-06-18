#include "ros/ros.h"
#include "demo/Person.h"

void dealMsg(const demo::Person &p){
    ROS_INFO("Received Massges, name=%s, id=%d deal by [CPP-demo03]", p.name.c_str(), p.id);
}


int main(int argc, char *argv[]){
    ROS_INFO("This is Subscriber [CPP-demo03]");
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建订阅者 & 关注话题
    ros::Subscriber sub = hd.subscribe<const demo::Person &>("stuinfo", 10, dealMsg);
    // 4.spin
    ros::spin();

    
    return 0;
}
