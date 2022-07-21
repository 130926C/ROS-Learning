#include "ros/ros.h"
#include "demo/addIntsAction.h"
#include "actionlib/server/simple_action_server.h"

typedef actionlib::SimpleActionServer<demo::addIntsAction> AcServer;


// 回调函数设计：
// 1. 解析提交的目标值
// 2. 产生连续反馈
// 3. 最终结果响应
// boost::function<void (const demo::addIntsGoalConstPtr &)> execute_callback
void Callbacks(
    const demo::addIntsGoalConstPtr &goal, 
    AcServer* server
){
    ros::Rate rate(10);
    int target_num = goal->target_num;
    ROS_INFO("Received target_num=%d", target_num);

    demo::addIntsFeedback feedback;

    int internal_result = 0;
    for(int i=1; i <= target_num; ++i){
        internal_result += i;
        feedback.prescent = ((double)i/target_num);
        // void AcServer::publishFeedback(const demo::addIntsFeedback &feedback)
        server->publishFeedback(feedback);
        rate.sleep();
    }
    demo::addIntsResult result;
    result.result_num = internal_result;

    server->setSucceeded(result);
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    // 创建server对象
    // SimpleActionServer(ros::NodeHandle n, 
    //     服务话题名      std::__cxx11::string name, 
    //     服务回调        boost::function<void (const demo::addIntsGoalConstPtr &)> execute_callback, 
    //     自动启动        bool auto_start);
    // 用 boost::bind 来绑定回调函数，相当于再嵌套一层
    AcServer server(hd, "add_server", boost::bind(&Callbacks, _1, &server) , true);
    
    server.start();
    ROS_INFO("Server Started Successed");
    ros::spin();
    return 0;
}
