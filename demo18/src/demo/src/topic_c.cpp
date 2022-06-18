#include "ros/ros.h"
#include "std_msgs/String.h"

// 设置不同类型的话题 

int main(int argc, char *argv[]){
    ROS_INFO("This is a topic node");
    // 1.初始化节点
    ros::init(argc, argv, "topic_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // A.全局话题：以 / 开头，此时和节点空间以及名字没有任何关系
    ros::Publisher pub_g = hd.advertise<std_msgs::String>("/global_topic", 10);
    // B.相对：直接写，此时和节点空间平级
    ros::Publisher pub_r = hd.advertise<std_msgs::String>("relative_topic", 10);
    // C.私有：需要创建特定 NodeHandle
    ros::NodeHandle nd("~");
    ros::Publisher pub_p = nd.advertise<std_msgs::String>("/private_topic", 10);


    // 3.操作
    while(ros::ok()){

    }

    

    return 0;
}
