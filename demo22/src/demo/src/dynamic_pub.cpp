#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h" // 坐标系转换头文件
#include "tf2/LinearMath/Quaternion.h"      // 四元数转化头文件

void doPose(const turtlesim::Pose::ConstPtr &pose){
    // 1.创建发布对象
    static tf2_ros::TransformBroadcaster pub;       // 这样使用回调函数使用的就是同一个对象
    // 2.组织发布数据
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";       // 相对坐标系的 id
    ts.header.stamp = ros::Time::now();
    ts.child_frame_id = "turtle1";      // 需要转化的坐标系 id
    // 坐标系偏移量
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0;
    // 坐标系四元数
    /*
        pose 中没有四元数，但有一个theat是偏航角度，同时乌龟是2D的，那么俯仰和翻滚也是0 
    */
    tf2::Quaternion que;
    que.setRPY(0, 0, pose->theta);          // 设置需要转换的坐标系
    ts.transform.rotation.w = que.getW();
    ts.transform.rotation.y = que.getY();
    ts.transform.rotation.z = que.getZ();
    ts.transform.rotation.x = que.getX();
    // 3.发布 
    pub.sendTransform(ts);
}



int main(int argc, char *argv[]){

    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.订阅乌龟的信息
    ros::Subscriber sub = hd.subscribe<const turtlesim::Pose::ConstPtr&>("turtle1/pose", 100, doPose);

    ros::spin();
    return 0;
}
