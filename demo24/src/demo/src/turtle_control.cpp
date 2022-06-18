/*
    
    控制乌龟运动
    1.换算出乌龟1相对于乌龟2的位置关系
    2.根据角速度和线速度进行运动

*/
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "control_node");
    // 2.获得句柄    
    ros::NodeHandle hd;

    // 3.创建一个坐标系信息的订阅者
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(2);

    // 4.创建用于发布乌控制龟运动的对象，将控制信息发布给turtle2的cmd_vel话题
    ros::Publisher pub = hd.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);


    while(ros::ok()){
        try{
            // 5.订阅者首先将乌龟1的坐标系转化为乌龟2的坐标系
            geometry_msgs::TransformStamped turtle2Toturtle1 = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
            /*
            ROS_INFO("Father:%s, Son:%s, bias:(%.2f, %.2f, %.2f)", 
                turtle2Toturtle1.header.frame_id.c_str(), 
                turtle2Toturtle1.child_frame_id.c_str(),
                turtle2Toturtle1.transform.translation.x,
                turtle2Toturtle1.transform.translation.y,
                turtle2Toturtle1.transform.translation.z
            );
            */

            // 准备一个控制乌龟运动的信息
            geometry_msgs::Twist twist;
            /*
                控制乌龟运动只需要设置twist.linear线速度的x和twist.angle角速度的z
                x = alpha * (y^2 + x^2)^0.5
                z = beta * arctan(c, b)
            */
            twist.linear.x = 0.5 * sqrt(pow(turtle2Toturtle1.transform.translation.x, 2) + pow(turtle2Toturtle1.transform.translation.y, 2));
            twist.angular.z = 0.5 * atan2(turtle2Toturtle1.transform.translation.y, turtle2Toturtle1.transform.translation.x);
            pub.publish(twist);

        }catch(const std::exception &e){
            ROS_ERROR("Error, %s", e.what());
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
