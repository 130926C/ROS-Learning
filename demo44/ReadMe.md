## demo 44

赵虚左课程中的 Action 部分内容，如果你之前是按照顺序阅读下来的话，那么这部分将比较轻松，否则建议还是先看看 demo31 部分的 Action 基础。

实现Action通讯需要完成以下三大步骤：
1. 定义action的信息载体 XXX.action；
2. 实现客户端；
3. 实现服务端；

【注意】Action底层是 **话题** 通讯，并不是server方式，因此在客户端上 ros::server::waitForServer() 会抛出异常

----

### **Step 1**：创建功能包 & 导入依赖
需要以下包作为依赖：
```txt
roscpp rospy std_msgs actionlib actionlib_msgs
```

1. 在demo包下面新建一个目录 action，并添加 addInts.action 文件；
```txt
# 目标数据
int32 target_num
---
# 最终反馈
int32 result_num
---
# 连续反馈
float32 prescent
```
2. 修改 CMakeList.txt 文件；
```txt
add_action_files(
  FILES
  addInts.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
)
```
3. 编译生成中间文件 & 配置 c_cpp_properties.json
```shell
$ catkin_make
```

```json
"includePath": [
    "/opt/ros/melodic/include/**",
    "/usr/include/**", 
    "/home/gaohao/Desktop/ROS-Learning/demo44/devel/include"
],
```
----

### **Step 2**：服务端实现

**server.cpp**
```cpp 
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
```
验证服务端是否正确：
```shell
$ roscore
$ source devel/setup.bash
$ rosrun demo server_node

【注意】下面的 pub 和 echo 数据类型都可以用 Tab 自动补全，不用手打 
$ rostopic pub -1 /add_server/goal demo/addIntsActionGoal "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    goal_id:
    stamp:
        secs: 0
        nsecs: 0
    id: ''
    goal:
    target_num: 200" 

$ rostopic echo /add_server/feedback 
    header: 
    seq: 3
    stamp: 
        secs: 1658394797
        nsecs: 788314070
    frame_id: ''
    status: 
    goal_id: 
        stamp: 
        secs: 1658394797
        nsecs: 488095568
        id: "/server_node-1-1658394797.488095568"
    status: 1
    text: "This goal has been accepted by the simple action server"
    feedback: 
    prescent: 0.0
```

----

### **Step 3**：客户端实现 

**client.cpp**

```cpp
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
```

