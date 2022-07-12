#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "demo/countAction.h"

/*
    设计逻辑：
        client一旦发出了请求，那么就应该等待server计算完成，在此期间无论是查看反馈还是什么操作都应该用const，
        这样可以防止client意外改动了server正在处理的值，即client只能查看不能修改
*/


typedef actionlib::SimpleActionClient<demo::countAction> Client;

// 请求的回调:Server开始处理请求的时候执行一次，由DealAction激活，只会用一次
void ActionStartCallback(){
    ROS_INFO("Server start work [message from Client]");
}

// 响应回调：Server在处理过程中的响应，由as->publishFeedback激活
void FeedbackCallback(
    const demo::countFeedbackConstPtr &feedback
){
    ROS_INFO("Client received feedback, value is %d", feedback->internal);
}

// 完成回调：Server完成一次请求后的回调，由as->setSucceeded()激活，只会用一次
void DoneRequestCallBack(
    const actionlib::SimpleClientGoalState& state,
    const demo::countResultConstPtr& result
){
    ROS_INFO("Server done with normal[message from Client]");
    ros::shutdown();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle hd;

    // 定义客户端
    Client client("action_move", true);
    client.waitForServer();

    // 创建一个请求对象
    demo::countGoal target;
    target.target_num = 100;    // 设置终点值

    // 绑定client的回调
    client.sendGoal(target, &DoneRequestCallBack, &ActionStartCallback, &FeedbackCallback);

    ros::spin();
    return 0;
}
