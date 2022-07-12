#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo/countAction.h"

/*
    设计逻辑：
        server一旦接受了client的请求，就因该将目标固定下来，在此期间不能轻易修改目标，
        因此对于server来说client的目标应该是只读的
*/


// 将下面的类重命名【这个声明一定放在最前面，后面都要用这个声明】
typedef actionlib::SimpleActionServer<demo::countAction> Server;

// 回调函数
void DealAction(const demo::countGoalConstPtr& target, Server* as){
    ros::Rate rate(1);                      // 设置运行效率
    demo::countFeedback feedback;           // 创建一个反馈
    ROS_INFO("Target number is %d, [message from server]", target->target_num);
    // 开始计数
    for(int i=0; i < target->target_num; ++i){
        feedback.internal = i;              // 更新反馈值
        as->publishFeedback(feedback);      // 发布反馈
        rate.sleep();
    }
    ROS_INFO("\tCount done");
    as->setSucceeded();                     // 发送结果
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    // 用 boost::bind 来设置回调函数
    //    boost:bind 最多传入9个参数，用 _1,_2,_3... 的方式表示占位符。
    //               定义 bind(func,_1,a) 
    //               调用 func(b,a) 相当于使用占位符先占据一个参数位，调用的时候再传入参数b，但参数a仍会传入
    Server server(hd, "action_move", boost::bind(&DealAction, _1, &server), false);
    server.start();    
    
    ros::spin();

    return 0;
}
