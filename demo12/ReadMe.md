### demo12

其他函数实验

-----

Experiment 1：手动关停节点，检查在shutdown后面的代码会不会被执行 
```cpp
ROS_INFO("Befort shutdown code");
ros::shutdown();
ROS_INFO("After shutdown code");
```

-----

Experiment 2：日志信息
```cpp
ROS_INFO("ROS INFO Message");      // 正常日志
ROS_DEBUG("ROS DEBUG Message");    // debug条件下的输出，如果直接运行的话不会显示
ROS_WARN("ROS WARN Message");      // 警告
ROS_ERROR("ROS ERROR Message");    // 错误
ROS_FATAL("ROS FATAL Message");    // 致命

ROS_INFO("after message info");
```

```shell
$ rosrun demo log_node 
[ INFO] [1650249500.397970624]: This is log message
[ INFO] [1650249500.409654624]: ROS INFO Message
[ WARN] [1650249500.410577970]: ROS WARN Message
[ERROR] [1650249500.410612640]: ROS ERROR Message
[FATAL] [1650249500.410637092]: ROS FATAL Message
[ INFO] [1650249500.410659988]: after message info
```
在ros中，无论是输出什么类型的日志都不会造成程序中断。程序中断的原因是代码逻辑问题，和使用哪种类型的日志消息无关。

【注意】如果使用的是std::cout 方式输出信息可能会导致其他节点的信息无法正常输出。
