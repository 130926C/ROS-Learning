#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"


int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "bag_node");
    // 2.获得句柄
    ros::NodeHandle hd;

    // 3.创建bag对象
    rosbag::Bag bag;
    // 4.打开文件流
    bag.open("/home/lucks/Desktop/code/ROS/demo25/src/demo/bags/test.bag", rosbag::BagMode::Write);
    // 5.向文件流中写数据
    std_msgs::String msg;
    msg.data = "hello, this is rosbag";
    /*
        参数1：话题
        参数2：时间戳
        参数3：消息
    */
    bag.write("/chatter", ros::Time::now(), msg);

    // 6.关闭文件
    bag.close();

    return 0;
}
