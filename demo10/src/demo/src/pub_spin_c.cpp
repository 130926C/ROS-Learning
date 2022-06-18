#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is spin and spinonce Publisher");
    // 1.初始化节点
    ros::init(argc, argv, "pub_spin_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建发布者
    ros::Publisher pub = hd.advertise<std_msgs::String>("hello", 10);

    // 4.发布信息
    std_msgs::String msgs;
    msgs.data = "hello, this is publisher";
    
    // 实验一：前面无回调条件
    // pub.publish(msgs);
    // ROS_INFO("Published msgs is %s", msgs.data.c_str());
    // // ros::spin();
    // ros::spinOnce();
    // ROS_INFO("Spin finished");

    // 实验二：在循环中
    while(ros::ok()){
        pub.publish(msgs);
        ROS_INFO("Published msgs is %s", msgs.data.c_str());
        // ros::spin();
        ros::spinOnce();
    }

    return 0;
}

