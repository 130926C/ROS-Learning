#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char *argv[]){
    ROS_INFO("This publisher with latch experiment node");
    // 1.初始化节点
    ros::init(argc, argv, "pub_latch_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建发布方
    ros::Publisher pub = hd.advertise<std_msgs::String>("hello", 10, true);

    // 4.发布信息
    ros::Rate rate(1);
    ros::Duration(2);
    int curIndex = 0;

    while(ros::ok()){
        std_msgs::String msgs;
        std::stringstream ss;
        ss << "Hello, this is " << curIndex;
        msgs.data = ss.str().c_str();
        ss.str() = "";


        if (curIndex <= 10) {
            pub.publish(msgs);
            ROS_INFO("Published info is %s", msgs.data.c_str());    
            curIndex++;
        }

        rate.sleep();
    }

    return 0;
}
