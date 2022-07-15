#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/static_transform_broadcaster.h"

/*
    创建两个坐标系，初始化的时候 static_tf_1 参考的是 world 坐标系；
                            static_tf_2 参考的是 static_tf_1 坐标系；
*/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "transform_pub_node");
    ros::NodeHandle hd;
    tf2::Quaternion que;
    que.setRPY(0,0,0);
    
    geometry_msgs::TransformStamped static_tf_1;
    static_tf_1.header.frame_id = "world";          // 指定坐标系1的参考坐标系
    static_tf_1.header.stamp = ros::Time::now();
    static_tf_1.child_frame_id = "static_tf_1";     // 指定坐标系名
    static_tf_1.transform.translation.x = 1000;     // 坐标系和参考坐标系的偏移量
    static_tf_1.transform.rotation.w = que.getW();

    geometry_msgs::TransformStamped static_tf_2;
    static_tf_2.header.frame_id = "static_tf_1";
    static_tf_2.header.stamp = ros::Time::now();
    static_tf_2.child_frame_id = "static_tf_2";
    static_tf_2.transform.translation.x = -1000;
    static_tf_2.transform.rotation.w = que.getW();

    tf2_ros::StaticTransformBroadcaster pub;        // 准备一个坐标系关系发布者 
    pub.sendTransform(static_tf_1);
    pub.sendTransform(static_tf_2);

    ros::spin();
    return 0;
}
