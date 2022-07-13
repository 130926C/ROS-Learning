#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    if (argc != 2){
        ROS_ERROR("Error, need publisher node ID(unsigned int)");
        return -1;
    }

    char nodeName[100];
    sprintf(nodeName, "pub_node_%d", atoi(argv[1]));

    ros::init(argc, argv, nodeName);
    ros::NodeHandle hd;

    ros::Publisher pubA = hd.advertise<std_msgs::String>("topicA", 10);
    ros::Publisher pubB = hd.advertise<std_msgs::String>("topicB", 10);

    ros::Rate rate(1);

    std_msgs::String msg;
    char message[100];
    while(ros::ok()){
        sprintf(message, "topic A message from pub %d node", atoi(argv[1]));
        msg.data = message;
        pubA.publish(msg);

        sprintf(message, "topic B message from pub %d node", atoi(argv[1]));
        msg.data = message;
        pubB.publish(msg);
        
        rate.sleep();
        ros::spinOnce();
    } 
    
    return 0;
}
