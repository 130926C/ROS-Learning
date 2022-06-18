#include "ros/ros.h"
#include "demo/Person.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is Publisher [CPP-demo03]");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.创建句柄
    ros::NodeHandle hd;
    // 3.创建发布者 & 注册话题
    ros::Publisher pub = hd.advertise<demo::Person>("stuinfo", 10);
    
    demo::Person p;
    p.name = "zhang3";
    p.id = 0;
    ros::Rate rate(1);
    ros::Duration(2);

    // 4.发布信息
    while(ros::ok()){
        pub.publish(p);
        p.id++;
        ROS_INFO("Message Published, name=%s, id=%d by [CPP-demo03]", p.name.c_str(), p.id);
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
