#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char  *argv[]){
    ros::init(argc, argv, "tf_sub_node");
    ros::NodeHandle hd;
    ros::Rate rate(1);

    // 准备一个坐标系监听对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 准备坐落在两个坐标系上的点 
    geometry_msgs::PointStamped tf1_point;
    geometry_msgs::PointStamped tf2_point;
    geometry_msgs::PointStamped world_point;        // 世界坐标系上的点

    tf1_point.header.frame_id = "dynamic_tf_1";
    tf2_point.header.frame_id = "dynamic_tf_2";


    while(ros::ok()){
        try{
            // 让点随机出现在自己坐标系上
            tf1_point.header.stamp = ros::Time();
            tf1_point.point.x = rand() % 100;
            tf1_point.point.y = rand() % 100;
            tf1_point.point.z = rand() % 100;

            tf2_point.header.stamp = ros::Time();
            tf2_point.point.x = rand() % 100;
            tf2_point.point.y = rand() % 100;
            tf2_point.point.z = rand() % 100;

            // dynamic_tf_1 -> world
            world_point = buffer.transform(tf1_point, "world");
            ROS_INFO("%s point [%.2f,%.2f,%.2f] in %s is [%.2f,%.2f,%.2f]", 
                tf1_point.header.frame_id.c_str(), 
                tf1_point.point.x, tf1_point.point.y, tf1_point.point.z,
                world_point.header.frame_id.c_str(),
                world_point.point.x, world_point.point.y, world_point.point.z
            );

            // dynamic_tf_2 -> world
            world_point = buffer.transform(tf2_point, "world");
            ROS_INFO("%s point [%.2f,%.2f,%.2f] in %s is [%.2f,%.2f,%.2f]", 
                tf2_point.header.frame_id.c_str(), 
                tf2_point.point.x, tf2_point.point.y, tf2_point.point.z,
                world_point.header.frame_id.c_str(),
                world_point.point.x, world_point.point.y, world_point.point.z
            );

            // dynamic_tf_1 -> dynamic_tf_2
            world_point = buffer.transform(tf1_point, "dynamic_tf_2");
            ROS_INFO("%s point [%.2f,%.2f,%.2f] in %s is [%.2f,%.2f,%.2f]",  
                tf1_point.header.frame_id.c_str(), 
                tf1_point.point.x, tf1_point.point.y, tf1_point.point.z,
                world_point.header.frame_id.c_str(),
                world_point.point.x, world_point.point.y, world_point.point.z
        );
        }catch(const std::exception & e){
            ROS_INFO(e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
