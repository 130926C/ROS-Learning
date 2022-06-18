#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

int main(int argc, char *argv[]){
    ROS_INFO("This is publisher [CPP-demo02]");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.申请句柄
    ros::NodeHandle hd;
    // 3.创建发布者 & 注册话题
    ros::Publisher pub = hd.advertise<std_msgs::String>("hello", 1);

    // 4.发布信息
    ros::Rate rate(1);
    ros::Duration(2);
    int curIndex = 0;

    std_msgs::String msg;
    std::stringstream ss;

    while(ros::ok()){
        ss.str("");     // 这里需要将流中的内容清空一次，否则会不断往流中写入数据
        ss << "Hello, message from [CPP-demo02] with index=" << curIndex++ << std::endl;
        msg.data = ss.str();
        ROS_INFO("Published Message is: %s [CPP-demo02]", msg.data.c_str());
        pub.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
