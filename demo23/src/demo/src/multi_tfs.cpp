#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"    // 这个必须包含，否则编译报错
#include "geometry_msgs/TransformStamped.h"

int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.获得句柄
    ros::NodeHandle hd;
    // 3.创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    // 4.转换

    // 准备一个在son1上的坐标点
    geometry_msgs::PointStamped psAtSon1;
    psAtSon1.header.stamp = ros::Time::now();
    psAtSon1.header.frame_id = "son1";
    psAtSon1.point.x = 1;
    psAtSon1.point.y = 2;
    psAtSon1.point.z = 3;

    ros::Rate rate(10);
    while(ros::ok()){
        try{
            /*  lookupTransform()
                param 1: 目标坐标系 (相对父级坐标系)
                param 2: 源坐标系  （需要转换的坐标系）
                param 3: 时间戳
                return : 被转化后的坐标系
            */

            // 1.计算son1相对于son2坐标系偏移了多少
            geometry_msgs::TransformStamped son1Toson2 =  buffer.lookupTransform("son2", "son1", ros::Time(0));
            ROS_INFO("Son1 relative Son2 msgs: [Father:%s, Son:%s], bias value (%.2f, %.2f, %.2f)", 
                son1Toson2.header.frame_id.c_str(), 
                son1Toson2.child_frame_id.c_str(),
                son1Toson2.transform.translation.x, 
                son1Toson2.transform.translation.y,
                son1Toson2.transform.translation.z 
            );

            // 2.计算son1中某个点在son2中的坐标值
            geometry_msgs::PointStamped psAtSon2 = buffer.transform(psAtSon1, "son2");
            ROS_INFO("point is Son1 is (%.2f, %.2f, %.2f); point in Son2 is (%.2f, %.2f, %.2f), relative system is: %s", 
                psAtSon1.point.x,
                psAtSon1.point.y,
                psAtSon1.point.z,

                psAtSon2.point.x,
                psAtSon2.point.y,
                psAtSon2.point.z, 
                psAtSon2.header.frame_id.c_str()
            );
        }catch(const std::exception & e){
            ROS_INFO("Error Msgs: %s", e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
