#include "ros/ros.h"
#include "std_msgs/String.h"

void dealMsg(const std_msgs::String &msgs){
    ROS_INFO("Received Msgs is: [%s] deal by [CPP]", msgs.data.c_str());
}

int main(int argc, char *argv[]){
    ROS_INFO("This is Subscriber [CPP]");
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建订阅者 & 关注话题
    ros::Subscriber sub = hd.subscribe<const std_msgs::String &>("hello", 10, dealMsg);
        // 注意：如果这里不使用自动推倒，那么需要完整写入回调函数的参数类型

    // 4.spin
    ros::spin();

    return 0;
}
