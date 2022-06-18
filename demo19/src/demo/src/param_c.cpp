#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is params node");
    // 1.初始化节点
    ros::init(argc, argv, "param_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // A.全局
    ros::param::set("/param_global", -1);
    // B.相对
    ros::param::set("param_relative", 500);
    // C.私有
    ros::param::set("~param_private", 3.14);

    // A.全局
    hd.setParam("/param_global", -1);
    // B.相对
    hd.setParam("param_relative", 500);
    // C.私有
    ros::NodeHandle nd("~");
    nd.setParam("param_private", 3.14);

    
    return 0;
}
