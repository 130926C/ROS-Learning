#include "ros/ros.h"
#include "demo/addIntsAction.h"
#include "actionlib/client/simple_action_client.h"

// 响应成功回调
void DoneCallback(
    const actionlib::SimpleClientGoalState &state,
    const demo::addIntsResultConstPtr &result
){
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Server Successed done require, result is %d", result->result_num);
    }else{
        ROS_INFO("Server Error");
    }
}

// 成功激活回调
void ActionCallback(){
    ROS_INFO("Server received Clinet Required");
}

// 持续返回回调
void FeedbackCallback(const demo::addIntsFeedbackConstPtr &feedback){
    ROS_INFO("Current precent=%.2f", feedback->prescent);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle hd;

    ros::Rate rate(1);

    actionlib::SimpleActionClient<demo::addIntsAction> AcClient(hd, "add_server");
    // ros::service::waitForService("add_server");  // 【注意】对于 action 而言，这个是不适用的
    AcClient.waitForServer();


    demo::addIntsGoal goal;
    goal.target_num = 10;
    // void sendGoal(
    //   请求       const demo::addIntsGoal &goal, 
    //   结束回调    boost::function<void (const actionlib::SimpleClientGoalState &state, const demo::addIntsResultConstPtr &result)> done_cb, 
    //   激活回调    boost::function<void ()> active_cb, 
    //   反馈回调    boost::function<void (const demo::addIntsFeedbackConstPtr &feedback)> feedback_cb);    
    AcClient.sendGoal(
        goal, 
        &DoneCallback, 
        &ActionCallback,
        &FeedbackCallback
    );

    ros::spin();
    return 0;
}
