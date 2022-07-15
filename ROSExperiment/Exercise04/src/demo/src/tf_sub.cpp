#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/*
    在 static_tf_1 和 static_tf_2 坐标系上创建两个点，
    算这两个点相对于 world 坐标系的位置；
*/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "static_point_sub");
    ros::NodeHandle hd;
    ros::Rate rate(1);

    geometry_msgs::PointStamped world_point;        // 存放转换后的世界坐标系的点
    geometry_msgs::PointStamped tf1_point;          // static_tf_1 坐标系上的点
    geometry_msgs::PointStamped tf2_point;          // static_tf_2 坐标系上的点

    // 声明三个点所在的坐标系
    tf1_point.header.frame_id = "static_tf_1";
    tf2_point.header.frame_id = "static_tf_2";

    // 创建坐标系转化对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    while(ros::ok()){
        try{
            // 动态变化两个点的值

            // static_tf_1 上的点在 world 坐标系上的位置
            tf1_point.header.stamp = ros::Time::now();
            tf1_point.point.x = rand() % 50;
            tf1_point.point.y = rand() % 100;
            world_point = buffer.transform(tf1_point, "world");
            ROS_INFO("static_tf_1 point [%.2f, %.2f] in %s tf is [%.2f, %.2f]", 
                tf1_point.point.x, tf1_point.point.y,
                world_point.header.frame_id.c_str(), 
                world_point.point.x, world_point.point.y
            );

            // static_tf_2 上的点在 world 坐标系上的位置
            tf2_point.header.stamp = ros::Time::now();
            tf2_point.point.x = rand() % 50;
            tf2_point.point.y = rand() % 100;
            world_point = buffer.transform(tf2_point, "world");
            ROS_INFO("static_tf_2 point [%.2f, %.2f] in %s tf is [%.2f, %.2f]", 
                tf2_point.point.x, tf2_point.point.y,
                world_point.header.frame_id.c_str(), 
                world_point.point.x, world_point.point.y
            );

            // static_tf_2 上的点在 static_tf_1 坐标系上的位置
            world_point = buffer.transform(tf2_point, "static_tf_1");
            ROS_INFO("static_tf_2 point [%.2f, %.2f] in %s is [%.2f, %.2f]", 
                tf2_point.point.x, tf2_point.point.y,
                world_point.header.frame_id.c_str(), 
                world_point.point.x, world_point.point.y
            );
        }   
        catch(const std::exception& e){
            ROS_INFO(e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
