#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

void PrintNodePositionInfo(
    const geometry_msgs::PointStamped& move_point, 
    const geometry_msgs::PointStamped& target_point
){
    ROS_INFO("%s tf point [%.2f, %.2f] in %s tf is [%.2f, %.2f]", 
        move_point.header.frame_id.c_str(),
        move_point.point.x, move_point.point.y,
        target_point.header.frame_id.c_str(),
        target_point.point.x, target_point.point.y
    );
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_sub_node");
    ros::NodeHandle hd;
    ros::Rate rate(1);
    
    // 准备一个需要计算的点和一个存放结果的点
    geometry_msgs::PointStamped move_point;
    geometry_msgs::PointStamped target_point;

    // 创建一个转换对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    while (ros::ok()){
        try{
            move_point.point.x = rand()%30;
            move_point.point.y = rand()%40;

            // static -> world
            move_point.header.frame_id = "static_tf";
            move_point.header.stamp = ros::Time();
            target_point = buffer.transform(move_point, "world");
            PrintNodePositionInfo(move_point, target_point);

            // dynamic -> world
            move_point.header.frame_id = "dynamic_tf";
            move_point.header.stamp = ros::Time();
            target_point = buffer.transform(move_point, "world");
            PrintNodePositionInfo(move_point, target_point);            

            // static -> dynamic
            move_point.header.frame_id = "static_tf";
            move_point.header.stamp = ros::Time();
            target_point = buffer.transform(move_point, "dynamic_tf");
            PrintNodePositionInfo(move_point, target_point);

            // dynamic -> static
            move_point.header.frame_id = "dynamic_tf";
            move_point.header.stamp = ros::Time();
            target_point = buffer.transform(move_point, "static_tf");
            PrintNodePositionInfo(move_point, target_point);
        }
        catch(const std::exception& e){
            ROS_INFO(e.what());
        }
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
