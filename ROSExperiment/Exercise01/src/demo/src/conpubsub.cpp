#include "ros/ros.h"
#include "std_msgs/String.h"

static std_msgs::String g_msgs;

void DealSubscribeTopicA(std_msgs::String msg){
    ROS_INFO("Con Pub and Sub subscirbe messages %s", msg.data.c_str());
    g_msgs.data = msg.data;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "con_pub_sub_node");
    ros::NodeHandle hd;
    ros::Rate rate(1);

    ros::Publisher pub = hd.advertise<std_msgs::String>("topicB", 10);
    ros::Subscriber sub = hd.subscribe("topicA", 10, DealSubscribeTopicA);

    char msg[100];
    std_msgs::String message;
    while(ros::ok()){
        try{
            sprintf(msg, "received [%s] and trans to topicB", g_msgs.data.c_str());
            message.data = msg;
            pub.publish(message);
        }catch(ros::Exception &e){
            // pass
        }
        ros::spin();
    }

    return 0;
}
