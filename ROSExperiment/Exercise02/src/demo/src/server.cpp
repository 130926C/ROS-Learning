#include "ros/ros.h"
#include "std_msgs/String.h"
#include "demo/echo.h"

bool DealEchoRequire(
    demo::echoRequest& req,
    demo::echoResponse& res
){
    ROS_INFO("Echo server received request %s", req.reqS.c_str());
    res.backS = req.reqS;
    return true;
}

bool DealSilenceRequire(
    demo::echoRequest& req,
    demo::echoResponse& res
){
    ROS_INFO("Silence server received request, [No Data]");
    res.backS = "NULL";
    return true;
}

int main(int argc, char *argv[]){
    if (argc != 2){
        ROS_ERROR("Need a params server ID(unsigned int)");
        return -1;
    }
    char serverNodeName[100];
    sprintf(serverNodeName, "server_node_%d", atoi(argv[1]));
    ros::init(argc, argv, serverNodeName);
    ros::NodeHandle hd;

    // add server
    ros::ServiceServer echoServer = hd.advertiseService("echo_server", DealEchoRequire);
    ros::ServiceServer SilienceServer = hd.advertiseService("silience_server", DealSilenceRequire);

    ros::spin();

    return 0;
}
