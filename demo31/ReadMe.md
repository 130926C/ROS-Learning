## demo31 

使用动作库服务器 **Actionlib** 来实双向Server现通讯机制。

这里参考了 **沐棋** 的文章[ROS入门之——action](https://blog.csdn.net/weixin_41995979/article/details/81519533)
以及 **Sherlock的程序人生** 的文章[ROS节点通信（三）action](https://www.cnblogs.com/sherlock-lin/p/14994114.html)

在ROS中，Server-Client 是一对多的同步通讯模式，这一点可以自己作实验来验证，让Server在处理一个请求的时候直接sleep，Client不做任何sleep持续地发送请求。然后可以发现，Server是单线程运行的，对于sleep期间的请求统一返回false。  

【注意】阻塞是Server阻塞，而不是Client阻塞

上述方式存在一个弊端：如果Server被一个请求占据太长的时间，那么当前和其他Client都无法在逻辑上持续，无法知道Server处理的进度。  

因此，ROS提供了动作库 **Actionlib** 来弥补上述缺陷，在 Server 处理的期间 Client 可以持续获得当前请求的处理进度，必要时可以直接中断这次操作；同时，当 Server 处理结束后也会反馈一个结果给 Client，这一模式叫做 “目标-反馈-结果”。

---

使用 Actionlib 需要定义 action。
```txt
tip:
    在ROS功能包中：
        msg/ ：我们需要把自己定义的.msg文件放在这里
        srv/：我们需要把自己定义的.srv文件放在这里
        action/：我们需要把自己定义的.action文件放在这里
```

同样，为了更切合实际，这里是后面再添加相关依赖，初始化package的时候仅添加 roscpp rospy std_msgs。

----

**Step1:创建目录**

在 demo 目录下新建 action 文件夹（这一点和 msg、srv一样）
```shell
demo
    ├── action
    ├── CMakeLists.txt
    ├── include
    │   └── demo
    ├── package.xml
    └── src
```

------

**Step2:创建action文件**

创建一个计数器 action ，从st一直加到end，这样就可以在中间时刻获得进度。

count.action
```txt
int32 end_num   # 目标
---
bool finish     # 结果
---
int32 internal  # 反馈
```

------

**Step3:编写CMakeLists.txt并编译生成头文件**
CMakeLists.txt
```txt
# 添加 actionlib
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
)

# 释放 add_action_file
add_action_files(
  FILES
  count.action
)

# 添加 actionlib_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

```shell
$ catkin_make
```

因为 action 要比 msg 和 srv 复杂，所以会生成很多文件，包括了很多生成的变量如：countGlobalPtr这些，这些变量的命名方式为 “文件名XXX”。

```

-------

**Step4:修改package.xml**

-------

**Step5:添加头文件路径**
.vscode/c_cpp_properties.json
```json
    "includePath": [
        "/opt/ros/melodic/include/**",
        "/usr/include/**",
        "/home/gaohao/Desktop/temp/devel/include",
      ],
```

-------

**Step6:Server**

设计逻辑：

server一旦接受了client的请求，就因该将目标固定下来，在此期间不能轻易修改目标，因此对于server来说client的目标应该是只读的。

Server由于只存在一个处理过程，因此回调函数也仅需要定义一个处理函数即可。

头文件
```cpp
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo/countAction.h"
```

重命名以便于后续使用
```cpp
// 【这个声明一定放在最前面，后面都要用这个声明】
typedef actionlib::SimpleActionServer<demo::countAction> Server;
```

用于处理的回调函数
```cpp
void DealAction(demo::countGoalConstPtr& target, Server* as){
    ros::Rate rate(1);                      // 设置运行效率
    demo::countFeedback feedback;           // 创建一个反馈
    ROS_INFO("Target number is %d", target->target_num);
    // 开始计数
    for(int i=0; i < target->target_num; ++i){
        feedback.internal = i;              // 更新反馈值
        as->publishFeedback(feedback);      // 发布反馈
        rate.sleep();
    }
    ROS_INFO("\tCount done");
    as->setSucceeded();                     // 发送结果
}
```

主函数
```cpp
int main(int argc, char *argv[]){
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    // 用 boost::bind 来设置回调函数
    //    boost:bind 最多传入9个参数，用 _1,_2,_3... 的方式表示占位符。
    //               定义 bind(func,_1,a) 
    //               调用 func(b,a) 相当于使用占位符先占据一个参数位，调用的时候再传入参数b，但参数a仍会传入
    Server server(hd, "action_move", boost::bind(&DealAction, _1, &server), false);
    server.start();    

    return 0;
}
```

------

**Step7:Client**

Client 因为被允许发送请求、查询返回、完成计算、中断计算，所以需要声明多个回调。

设计逻辑：

client一旦发出了请求，那么就应该等待server计算完成，在此期间无论是查看反馈还是什么操作都应该用const，这样可以防止client意外改动了server正在处理的值，即client只能查看不能修改。

头文件：
```cpp
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo/countAction.h"
```

声明变量
```cpp
typedef actionlib::SimpleActionClient<demo::countAction> Client;
```

请求的回调：Server开始处理请求的时候执行一次，由DealAction激活，只会用一次
```cpp
void ActionStartCallback(){
    ROS_INFO("Server start work [message from Client]");
}
```

响应回调：Server在处理过程中的响应，由as->publishFeedback激活
```cpp
void FeedbackCallback(
    const demo::countFeedbackConstPtr &feedback
){
    ROS_INFO("Client received feedback, value is %d", feedback->internal);
}
```

完成回调：Server完成一次请求后的回调，由as->setSucceeded()激活，只会用一次
```cpp
void DoneRequestCallBack(
    const actionlib::SimpleClientGoalState& state,
    const demo::countResultConstPtr& result
){
    ROS_INFO("Server done with normal[message from Client]");
    ros::shutdown();
}
```

主函数
```cpp
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

```

----

**Step8:修改CMakeLists.txt**

添加包find_package
```txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
)
```

添加action文件
```txt
add_action_files(
  FILES
  count.action
)
```

添加生成信息
```txt
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
```

增加可执行文件
```txt
add_executable(server_node src/server.cpp)
add_executable(client_node src/client.cpp)
```

增加依赖文件
```txt
add_dependencies(server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

添加动态链接库
```txt
target_link_libraries(server_node
  ${catkin_LIBRARIES}
)
target_link_libraries(client_node
  ${catkin_LIBRARIES}
)
```