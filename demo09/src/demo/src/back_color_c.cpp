#include "ros/ros.h"
#include "turtlesim/Color.h"

#include "std_srvs/Empty.h"

int main(int argc, char *argv[]){
    ROS_INFO("This is turtle background color demo");
    // 1.初始化节点
    ros::init(argc, argv, "color_node");

    // 2.修改参数
    ros::param::set("/turtlesim/background_r", 0);
    ros::param::set("/turtlesim/background_g", 0);
    ros::param::set("/turtlesim/background_b", 0);

    // 3.动态刷新(解决思路看当前文件夹下的 ReadmMe.md文件)
    // (1) 创建句柄
    ros::NodeHandle hd;
    // (2) 创建client
    ros::ServiceClient client = hd.serviceClient<std_srvs::Empty>("/clear");
    // (3) 创建一个数据请求
    std_srvs::Empty empty;
    client.waitForExistence();
    // (4) 发送请求
    client.call(empty);

    // 此时乌龟的背景颜色就被刷新了，而无需重新启动

    return 0;
}
