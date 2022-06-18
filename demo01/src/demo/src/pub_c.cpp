#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is Publisher [CPP]");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建发布者
    ros::Publisher pub = hd.advertise<std_msgs::String>("hello", 10);
    // 4.发布信息
    std_msgs::String msgs;
    msgs.data = "Hello, this is publisher from demo01 send by [CPP]";
    ros::Duration(2);
    ros::Rate rate(1);
    while(ros::ok()){
        pub.publish(msgs);
        ROS_INFO("Published msgs is:[%s] [CPP]", msgs.data.c_str());
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
