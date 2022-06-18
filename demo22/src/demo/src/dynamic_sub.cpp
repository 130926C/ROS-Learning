#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"        // 必须添加这个头文件，否则会编译报错

int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "sub_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.创建一个buffer
    tf2_ros::Buffer buffer;
    // 4.创建坐标的listener 并传入 buffer
    tf2_ros::TransformListener listener(buffer);
    // 5.创建一个被转换的坐标点
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "turtle1";
    ps.header.stamp = ros::Time(0.0);       // 这里不能用 now

    ps.point.x = 2.0;
    ps.point.y = 1.0;
    ps.point.z = 0.0;
    
    ros::Rate rate(10);

    while(ros::ok()){
        geometry_msgs::PointStamped ps_out;
        try{
            ps_out = buffer.transform(ps, "world");     // 被转换的坐标系是绝对坐标系
            ROS_INFO("Transled point=(%.2f, %.2f, %.2f), relative system=%s",
                ps_out.point.x,
                ps_out.point.y,
                ps_out.point.z,
                ps_out.header.frame_id.c_str()
            );
        }catch(const std::exception &e){
			ROS_INFO("Exception Message, %s", e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
