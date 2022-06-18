#include "ros/ros.h"
#include "std_msgs/String.h"

void dealMsgs(const std_msgs::String msgs){
    ROS_INFO("Received Msgs is %s", msgs.data.c_str());
}


int main(int argc, char  *argv[]){
    ROS_INFO("This is Sub node");
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建订阅者 & 订阅话题
    ros::Subscriber sub = hd.subscribe<const std_msgs::String>("pub_topic", 10, dealMsgs);
    // 4.spin
    ros::spin();

    return 0;
}
