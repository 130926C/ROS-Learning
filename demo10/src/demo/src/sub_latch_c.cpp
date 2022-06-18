#include "ros/ros.h"
#include "std_msgs/String.h"

void dealMsgs(const std_msgs::String &msg){
    ROS_INFO("Subscribed Msgs is %s", msg.data.c_str());
}

int main(int argc, char *argv[]){
    ROS_INFO("This is sublisher with latch");
    // 1.初始化节点
    ros::init(argc, argv, "sub_latch_node", ros::init_options::AnonymousName);
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.订阅话题
    ros::Subscriber sub = hd.subscribe<const std_msgs::String &>("hello", 10, dealMsgs);

    // 4.spin
    ros::spin();

    return 0;
}
