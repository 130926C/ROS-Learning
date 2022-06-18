#include "ros/ros.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is Paramer OP [CPP-demo05]");
    // 1.初始化节点
    ros::init(argc, argv, "param_op_node");
    // 2.创建句柄
    ros::NodeHandle hd;
   
    // 3.添加参数
    hd.setParam("heigth", 10);
    hd.setParam("width", 12);

    // 4.获得所有参数名
    std::vector<std::string> pNames;
    hd.getParamNames(pNames);
    for (auto &&name: pNames){
        ROS_INFO("Params Name: %s", name.c_str());
    }


    return 0;
}
