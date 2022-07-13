#include "ros/ros.h"
#include "std_msgs/String.h"

void DealSubscribeTopicA(std_msgs::String msg){
    ROS_INFO("Subscribed Message %s", msg.data.c_str());
}

void DealSubscribeTopicB(std_msgs::String msg){
    ROS_INFO("Subscribed Message %s", msg.data.c_str());
} 

int main(int argc, char *argv[]){
    if (argc != 2){
        ROS_ERROR("Error, need subscriber node ID(unsigned int)");
        return -1;
    }

    char nodeName[100];
    sprintf(nodeName, "sub_node_%d", atoi(argv[1]));
    ros::init(argc, argv, nodeName);
    ros::NodeHandle hd;

    ros::Subscriber subA = hd.subscribe("topicA", 10, DealSubscribeTopicA);
    ros::Subscriber subB = hd.subscribe("topicB", 10, DealSubscribeTopicB);
    ros::spin();

    return 0;
}
