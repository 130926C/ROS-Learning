#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/*
    已知：
        1.节点/teleop_tutle通过话题turtle1/cmd_vel给节点/turtlesim传递信息
        2.话题turtle1/cmd_vel的数据类型是geometry_msgs/Twist
        3.结构体geometry_msgs/Twist有以下几个内容：
            linear:
                float64 x -> 向前
                float64 y
                float64 z
            angular:
                float64 x
                float64 y -> 左右
                float64 z 
    要求：
        实现小乌龟的圆周运动

*/

int main(int argc, char *argv[]){
    ROS_INFO("This is auto control [CPP]");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建发布者
    ros::Publisher pub = hd.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    // 4.发布信息

    geometry_msgs::Twist geoT;
    geoT.linear.x = 1;
    geoT.angular.z = 0.5;

    ros::Rate rate(1);
    ros::Duration(2);

    while(ros::ok()){
        pub.publish(geoT);
        ROS_INFO("Control Publish successed");
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
