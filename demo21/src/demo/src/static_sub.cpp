#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[]){
    // 1.初始化
    ros::init(argc, argv, "sub_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.创建订阅对象 & 将buffer绑定到订阅对象上
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 3.创建一个座标点(雷达采集到的坐标点，雷达是坐标原点)
    geometry_msgs::PointStamped pst;
    pst.header.frame_id = "laser";
    pst.header.stamp = ros::Time::now();

    pst.point.x = 1.0;      // 实际场景下这个坐标点是循环更新的，这里把他写死了
    pst.point.y = 2.3;
    pst.point.z = 0.2;

    // 这里有一个点需要注意：
    // 因为ros是并行运行的，所以可能会出现订阅方已经启动了但发布方还没发布消息，可以通过下面两个方法解决：
    // 1.添加休眠
    // ros::Duration(2).sleep();   // 休眠2s
    // 2.处理异常

    ros::Rate rate(10);
    while(ros::ok()){
        // 4.将雷达采集到的坐标点转换成base_link的位置
        geometry_msgs::PointStamped ps_out;         // 转换后的结果

        try{
            ps_out = buffer.transform(pst, "base_link");
            ROS_INFO("Transled point=(%.2f, %.2f, %.2f), relative system=%s", 
                ps_out.point.x,
                ps_out.point.y,
                ps_out.point.z,
                ps_out.header.frame_id.c_str()
            );
        }
        catch(const std::exception& e){
            ROS_INFO("Exception Message, %s", e.what());
        }

        rate.sleep();
        ros::spinOnce();
    }



    return 0;
}
