#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"


int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_pub_node");
    ros::NodeHandle hd;
    ros::Rate rate(1);
    tf2::Quaternion que;
    que.setRPY(0,0,0);

    // 准备一个动态坐标系发布对象
    tf2_ros::TransformBroadcaster tf_pub;
    
    // 准备连个动态坐标系对象 
    geometry_msgs::TransformStamped dynamic_tf_1;
    geometry_msgs::TransformStamped dynamic_tf_2;

    // 给坐标系绑定参考坐标系
    dynamic_tf_1.header.frame_id = "world";
    dynamic_tf_1.child_frame_id = "dynamic_tf_1";

    dynamic_tf_2.header.frame_id = "dynamic_tf_1";      // 让 dynamic_tf_2 以 dynamic_tf_1 为参考
    dynamic_tf_2.child_frame_id = "dynamic_tf_2";

    while(ros::ok()){
        // 随机传入坐标系位置
        dynamic_tf_1.header.stamp = ros::Time::now();       // 因为坐标系是动态的，所以时间戳一定需要是动态的
        dynamic_tf_1.transform.translation.x = rand() % 1000 + 1000;
        dynamic_tf_1.transform.translation.y = rand() % 1000 + 2000;
        dynamic_tf_1.transform.translation.z = rand() % 1000 + 3000; 
        dynamic_tf_1.transform.rotation.w = que.getW();
        
        dynamic_tf_2.header.stamp = ros::Time::now();
        dynamic_tf_2.transform.translation.x = rand() % 100 + 100;
        dynamic_tf_2.transform.translation.x = rand() % 100 + 200;
        dynamic_tf_2.transform.translation.x = rand() % 100 + 200;       
        dynamic_tf_2.transform.rotation.w = que.getW();

        // 发布
        tf_pub.sendTransform(dynamic_tf_1);
        tf_pub.sendTransform(dynamic_tf_2);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
