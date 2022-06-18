#include "ros/ros.h"
#include "std_msgs/String.h"

void dealMsg(const std_msgs::String &msg){
    ROS_INFO("Received Message is :%s. deal by [CPP-demo02]", msg.data.c_str());
}

int main(int argc, char *argv[]){
    ROS_INFO("This is subcrisber [CPP-demo02].");
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.申请句柄
    ros::NodeHandle hd;
    // 3.创建订阅者& 关注话题
    ros::Subscriber sub = hd.subscribe<const std_msgs::String &>("hello", 10, dealMsg);
    // 4.Spin
    ros::spin();


    return 0;
}
