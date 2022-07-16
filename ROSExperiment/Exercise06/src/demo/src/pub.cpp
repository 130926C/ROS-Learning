#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_pub_node");
    ros::NodeHandle hd;
    ros::Rate rate(1);
    tf2::Quaternion que;
    que.setRPY(0, 0, 0);

    tf2_ros::StaticTransformBroadcaster static_pub;
    tf2_ros::TransformBroadcaster dynamic_pub;

    // 定义一个静态坐标系
    geometry_msgs::TransformStamped static_tf;
    static_tf.header.frame_id = "world";
    static_tf.header.stamp = ros::Time::now();
    static_tf.child_frame_id = "static_tf";
    static_tf.transform.translation.x = 10;
    static_tf.transform.rotation.w = que.getW();
    static_pub.sendTransform(static_tf);            // 发布静态坐标系 

    // 定义一个动态坐标系
    geometry_msgs::TransformStamped dynamic_tf;
    dynamic_tf.header.frame_id = "static_tf";
    dynamic_tf.child_frame_id = "dynamic_tf";

    while(ros::ok()){
        try{           
            dynamic_tf.header.stamp = ros::Time::now();
            dynamic_tf.transform.translation.x = rand() % 30;
            dynamic_tf.transform.rotation.w = que.getW();
            dynamic_pub.sendTransform(dynamic_tf);   
        }
        catch(const std::exception& e){
            ROS_INFO(e.what());
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
