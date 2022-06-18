#include "ros/ros.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is a ROS node with argv");
    // 1.促使化节点
    ros::init(argc, argv, "stp_node");
    // 2.给参数服务器设置参数
    if (argc == 3){
        ros::param::set(argv[1], argv[2]);
    }else{
        ROS_INFO("Param must equal to 2, [key] and [value]");
        return -1;
    }
    // 3.获得句柄
    ros::NodeHandle hd;

    // 4.查看参数服务器内容
    std::vector<std::string>paramNameList;
    hd.getParamNames(paramNameList);
    for (auto &&names : paramNameList){
        // ROS_INFO("Param: %s = %s", names.c_str(), hd.getParam(names));
        std::string values;
        hd.getParam(names, values);
        ROS_INFO("Param %s = %s", names.c_str(), values.c_str());
    }
    
    return 0;
}
