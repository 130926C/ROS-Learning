#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]){
    // 1.初始化节点
    ros::init(argc, argv, "read_node");
    // 2.获得句柄
    ros::NodeHandle hd;
    // 3.创建bag对象
    rosbag::Bag bag;
    // 4.打开文件
    bag.open("/home/lucks/Desktop/code/ROS/demo25/src/demo/bags/test.bag", rosbag::BagMode::Read);
    // 5.循环读入消息
    // 这里也可以使用 auto &&m
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        // 获取话题
        std::string topic = m.getTopic();  
        // 获取时间戳
        ros::Time time = m.getTime();
        // 获取消息（instantiate返回的是一个指针类型的数据）
        std_msgs::StringConstPtr msg = m.instantiate<std_msgs::String>();
        ROS_INFO("Topic:%s, Time:%.2f, Msgs:%s", 
            topic.c_str(), 
            time.toSec(), 
            msg->data.c_str()
        );
    }

    // 6.关闭文件
    bag.close();

    return 0;
}
