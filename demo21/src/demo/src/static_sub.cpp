#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/*
    可以发现，从始自终我们都没有给world坐标系设置任何参数，仅仅给了这个坐标系一个名字“world”
    TF tree 会根据这个名字来寻找坐标系，可以将其假想成一个“绝对”坐标系，而绝对坐标系就是（0，0，0），所以不需要任何参数，
    但是world坐标系也有可能有一个 header.frame_id
*/

int main(int argc, char *argv[]){
    // 1.初始化
    ros::init(argc, argv, "sub_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.创建订阅对象 & 将buffer绑定到订阅对象上
    //   后面需要从buffer中拿取坐标系
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 3.创建一个雷达采集到点，这个点是从雷达坐标系原点的角度出发的 
    geometry_msgs::PointStamped laser_cap_point;
    //   【注意】这里的 header.frame_id 和 坐标系的不是一个东西，
    //          其实也可以发现 laser_cap_point 只有 header和point 两个成员
    //          说明 laser_cap_point 的 header.frame_id 指的就是该点所在的坐标系名
    laser_cap_point.header.frame_id = "laser";          // 这个点的坐标系是laser
    laser_cap_point.header.stamp = ros::Time::now();

    // 假设雷达得到了下面这个点，因为点没有方向的概念，所以不会有旋转
    laser_cap_point.point.x = 1.0;
    laser_cap_point.point.y = 2.3;
    laser_cap_point.point.z = 0.2;

    // 这里有一个点需要注意：
    // 因为ros是并行运行的，所以可能会出现订阅方已经启动了但发布方还没发布消息，可以通过下面两个方法解决：
    // 1.添加休眠
    // ros::Duration(2).sleep();   // 休眠2s
    // 2.处理异常

    ros::Rate rate(10);
    // 4.将雷达采集到的点映射在world坐标系上的点
    geometry_msgs::PointStamped world_point;   
    while(ros::ok()){
        try{
            // 对点进行转化，将点转化成world坐标系上 
            world_point = buffer.transform(laser_cap_point, "world");
            ROS_INFO("origin TF point=(%.2f, %.2f, %.2f);\nworld TF point=(%.2f, %.2f, %.2f), world transform name=%s", 
                laser_cap_point.point.x,
                laser_cap_point.point.y,
                laser_cap_point.point.z,
                world_point.point.x,
                world_point.point.y,
                world_point.point.z,
                world_point.header.frame_id.c_str()
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
