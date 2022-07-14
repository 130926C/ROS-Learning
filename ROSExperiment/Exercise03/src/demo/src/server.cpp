#include "ros/ros.h"
#include "std_msgs/String.h"
#include "demo/count.h"
#include <thread>
#include <iostream>

bool DealCountRequirest(
    demo::countRequest &req, 
    demo::countResponse &res
){ 
    ros::Rate rate(1);
    std::thread::id tid = std::this_thread::get_id();
    std::cout << "\tThread " << tid << " processing " << req.clientID << " client ID"<< std::endl;
    for(int i = req.num1; i < req.num2; ++i){
        ROS_INFO("Client ID=%d, value=%d", req.clientID, i);
        rate.sleep();
    }
    res.result = -1;

    return true;
}


int main(int argc, char *argv[]){
    if (argc != 2){
        ROS_INFO("Need spin type ID(unsigned int)");
        ROS_INFO("\tros::spin() 0\n\tros::spinOnce()\t1\n\tros::MultiThreadedSpinner()\t2\n\tros::AsyncSpinner\t3\t");
    }
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    ros::MultiThreadedSpinner mTSpinner(4);
    ros::AsyncSpinner aSSpinner(4);

    ros::ServiceServer server = hd.advertiseService("count_server", DealCountRequirest);
    
    switch (atoi(argv[1]))
    {
    case 0:
        ros::spin();break;
    case 1:
        ros::spinOnce(); break;
    case 2:
        mTSpinner.spin(); break;
    case 3:
        aSSpinner.start(); break;
    default:
        ros::spinOnce(); break;
    }

    return 0;
}
