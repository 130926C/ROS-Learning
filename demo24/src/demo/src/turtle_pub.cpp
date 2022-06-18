/*
    发布两个乌龟在world坐标系下的坐标点信息

    1.订阅到两个乌龟的pose信息;
    2.将pose信息转化为wordl坐标系下的坐标点;
    3.发布这个坐标点;

*/

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/subscriber.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"

static std::string turtle_name;


// 这个函数用来处理pose信息，根据pose信息将坐标系进行转化
void doPoseMsgs(const turtlesim::Pose & pose){
    // 1.创建一个坐标系
    geometry_msgs::TransformStamped tfs;
    // 2.将pose信息潜入到这个坐标系中
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name;

    tfs.transform.translation.x = pose.x;
    tfs.transform.translation.y = pose.y;
    tfs.transform.translation.z = 0.0;

    // 3.转化四元数
    tf2::Quaternion qtu;
    qtu.setRPY(0, 0, pose.theta);
    tfs.transform.rotation.x = qtu.getX();
    tfs.transform.rotation.y = qtu.getY();
    tfs.transform.rotation.z = qtu.getZ();
    tfs.transform.rotation.w = qtu.getW();

    // 4.创建一个坐标系发布者(这里需要做成静态的，否则每都会申请导致 /tf 话题无法持续)
    // 这个发布者必须要先初始化节点之后才能创建
    static tf2_ros::TransformBroadcaster pub;

    // 5.将信息发布
    pub.sendTransform(tfs);
}


int main(int argc, char *argv[]){
    // 1.初始化节点    
    ros::init(argc, argv, "tf_node");
    // 2.获得句柄
    ros::NodeHandle hd;
    // 3.检查参数合法性
    if (argc != 2){
        ROS_ERROR("Error: param should be one");
        return 1;
    }
    turtle_name = argv[1];

    // 4.订阅pose话题
    ros::Subscriber sub = hd.subscribe<const turtlesim::Pose &>(turtle_name+"/pose", 100, doPoseMsgs);
    ros::spin();

    return 0;
}
