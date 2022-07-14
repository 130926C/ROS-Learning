#include "ros/ros.h"
#include "demo/count.h"

int main(int argc, char *argv[]){
    if (argc != 2){
        ROS_ERROR("Error, need a param ID (unsigned int)");
        return -1;
    }
    char nodeName[100];
    sprintf(nodeName, "client_node_%d", atoi(argv[1]));
    ros::init(argc, argv, nodeName);
    ros::NodeHandle hd;

    ros::Rate rate(1);
    demo::count req;
    req.request.clientID = atoi(argv[1]);

    ros::ServiceClient client = hd.serviceClient<demo::count&>("count_server");
    ros::service::waitForService("count_server");

    while (ros::ok()){
        req.request.num1 = atoi(argv[1]) * 10;
        req.request.num2 = atoi(argv[1]) * 20 + 10;
        client.call(req);
        rate.sleep();
    }

    return 0;
}
