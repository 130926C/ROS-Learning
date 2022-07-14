#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    由于坐标变化涉及到非常多的数学运算，所以ROS采用了坐标树 TF tree 的形式来管理被发布出去的坐标。
    
    如果想要在话题中发布一个点的空间位置，就必须带着它自己的坐标系发布出去，这样方便后面的坐标转化。

    因为使用欧拉角描述物体的空间旋转存在万向节死锁的情况，所以ROS不允许空间点使用直接用欧拉角，
    然而欧拉角才是人类能理解的空间旋转方式（翻滚、偏航、俯仰），所以ROS提供了欧拉角转成四元数的包。

    所以应当先用欧拉角描述点旋转情况，然后转化成四元数再布出去

    那么这个pub发布的其实是一个坐标系信息，sub从话题中拉取这个坐标系信息然后再解析
*/


int main(int argc, char  *argv[]){
    ROS_INFO("This is Static Transform demo");
    // 1.初始化节点
    ros::init(argc, argv, "pub_node");
    // 2.获得句柄
    ros::NodeHandle hd;
    // 3.获得发布者对象
    //   因为ROS使用的是 TF tree 来管理所有坐标系，所以需要一个特殊的发布者
    tf2_ros::StaticTransformBroadcaster pub;

    // 4.创建一个坐标系
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";          // 当前坐标系参考的坐标系名
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "laser";           // 当前坐标系名

    // 5.当前坐标系原点相对于世界坐标系的位移关系
    //   ROS中不推荐使用欧拉角描述旋转量，所以需要创建一个“欧拉角-四元数”转化器   
    tf2::Quaternion quate;
    //   Roll(x), Pitch(y), Yaw(z) -> RPY
    quate.setRPY(1.25, 3.00, 0.51);

    // 7.将四元数信息附加到坐标系上
    tfs.transform.rotation.x = quate.getX();
    tfs.transform.rotation.y = quate.getY();
    tfs.transform.rotation.z = quate.getZ();
    tfs.transform.rotation.w = quate.getW();

    // 8.发布坐标系到 TF tree 上
    pub.sendTransform(tfs);
    ros::spin();

    return 0;
}