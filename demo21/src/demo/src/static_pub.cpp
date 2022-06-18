#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char  *argv[]){
    ROS_INFO("This is Static Transform demo");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.获得句柄
    ros::NodeHandle hd;
    // 3.获得发布者对象
    tf2_ros::StaticTransformBroadcaster pub;

    // 4.创建需要发布的信息
    geometry_msgs::TransformStamped tfs;
    // 5.声明零坐标系
    tfs.header.frame_id = "base_link";    // 被参考的坐标系名
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "laser";

    // 6.相对坐标系原点的空间位置
    tfs.transform.translation.x = 3.4;
    tfs.transform.translation.y = 2.1;
    tfs.transform.translation.z = 0.8;

    // 7.设置这个坐标系的旋转量（如果不想旋转的话设置成0，0，0也可以）
    tf2::Quaternion quate;
    quate.setRPY(1.25, 3.00, 0.51);             // 设置在相对坐标系上的反转量

    // 8.转化成四元数给信息 （注:即便不旋转也要设置，因为发布的信息必须完整）
    tfs.transform.rotation.x = quate.getX();
    tfs.transform.rotation.y = quate.getY();
    tfs.transform.rotation.z = quate.getZ();
    tfs.transform.rotation.w = quate.getW();

    // 9.发布信息
    pub.sendTransform(tfs);
    ros::spin();

    return 0;
}
