#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is Pub node");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.获得句柄 
    ros::NodeHandle hd;
    // 3,创建发布者 & 订阅话题
    ros::Publisher pub = hd.advertise<std_msgs::String>("pub_topic", 10);

    // 4.发布话题
    std_msgs::String msgs;
    msgs.data = "This data is from ros pub node";
    ros::Rate rate(1);
    ros::Duration(2);
    while(ros::ok()){
        pub.publish(msgs);
        ROS_INFO("Published Message is %s", msgs.data.c_str());
        ROS_INFO("published time is %.3f", ros::Time::now().toSec());
        rate.sleep();
    }

    return 0;
}
