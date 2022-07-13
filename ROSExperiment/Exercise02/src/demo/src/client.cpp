#include "ros/ros.h"
#include "std_msgs/String.h"
#include "demo/echo.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle hd;

    ros::ServiceClient echoClient = hd.serviceClient<demo::echo>("echo_server");
    ros::ServiceClient silienceClient = hd.serviceClient<demo::echo>("silience_server");

    demo::echo msg;
    
    msg.request.reqS = "client echo require";
    ros::Rate rate(1);

    while(ros::ok()){
        if (echoClient.call(msg)){
            ROS_INFO("Echo server working true");
        }
        if (silienceClient.call(msg)){
            ROS_INFO("Silience server working true");
        }
        rate.sleep();
    }

    return 0;
}
