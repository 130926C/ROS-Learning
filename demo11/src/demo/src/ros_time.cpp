#include "ros/ros.h"

void TimerCb(const ros::TimerEvent& event){
    ROS_INFO("ROS Timer Callback at %.2f", ros::Time::now().toSec());
    ROS_INFO("Current Time: %.2f", event.current_real.toSec());
}

int main(int argc, char *argv[]){
    ROS_INFO("This is time demo get and set");
    // 1.初始化节点
    ros::init(argc, argv, "time_node");
    // 2.创建句柄(在)
    ros::NodeHandle hd;
    
    // 3.获得当前时刻
    ros::Time right_now = ros::Time::now();
    ROS_INFO("Seconds: %.3f", right_now.toSec()); 
    ROS_INFO("Seconds: %d", right_now.sec);
    ROS_INFO("-------------------------------------");

    // 4.设置指定时刻
    ros::Time t1(53, 124131);
    ROS_INFO("Time: %.2f", t1.toSec());
    ROS_INFO("-------------------------------------");


    // 5.休眠时间
    ROS_INFO("Program Sleeping at %.3f", ros::Time::now().toSec());
    ros::Duration dur(1.5);
    dur.sleep();
    ROS_INFO("Program Wake at %.3f", ros::Time::now().toSec());
    ROS_INFO("-------------------------------------");


    // 6.计算程序结束时间
    // 方法一：时间的可加性
    ros::Time ts(2.4);
    ROS_INFO("Calc Start Time: %.2f; End Time: %.2f", ros::Time::now().toSec(), ros::Time::now().toSec() + ts.toSec());
    // 方法二：模拟
    ros::Duration durs(ts.toSec());
    t1 = ros::Time::now();
    durs.sleep();
    ROS_INFO("Simu Start Time: %.2f; End Time: %.2f", t1.toSec(), ros::Time::now().toSec());
    ROS_INFO("-------------------------------------");


    // 7,设置定时器回调
    ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb);
    ros::spin();        // 上文有回调函数满足spin条件

    // 8.定时器只启动一次
    ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb, true);

    // 9.定时器以手动方式启动
    ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb, false, false);
    timer.start();
    ros::spin();

    return 0;
}
