#include "ros/ros.h"
#include <string>

int main(int argc, char *argv[]){
    ROS_INFO("This is launch node");
    // 1.初始化节点
    ros::init(argc, argv, "launch_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.获得参数服务器的内容
    std::vector<std::string> parNameList;
    ros::param::getParamNames(parNameList);

    for (auto &name: parNameList){
        bool flag = true;
        std::string value;

        flag = ros::param::get(name.c_str(), value);
        ROS_INFO("Param:%s, value[%d]=%s", name.c_str(), flag,value.c_str());           
    }

    return 0;
}
