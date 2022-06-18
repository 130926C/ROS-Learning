### demo26

rqt工具箱是ROS提供的一种基于QT实现的图形化界面，如果安装的是desktop-full版本的话是自带的，否则可以通过以下命令进行安装：
```shell
$ sudo apt-get install ros-noetic-rqt
$ sudo apt-get install ros-noetic-rqt-common-plugins
```

启动（二选一）：
```shell
$ rqt
$ rosrun rqt_gui rqt_gui
```

-------

### rqt-concle查看日志信息

准备一个在控制台输出的文件
```cpp
#include "ros/ros.h"

int main(int argc, char *argv[]){
	// 1.初始化节点
	ros::init(argc, argv, "log_node");
	// 2.获得句柄
	ros::NodeHandle hd;
	
	// 3.输出日志
	ros::Rate rate(0.3);
	int count = 0;
	
	while(ros::ok()){
		ROS_INFO("Count:%d message", count);
		ROS_DEBUG("ROS DEBUG MESSAGE");
		ROS_INFO("ROS INFO MESSAGE");
		ROS_WARN("ROS WARNING MESSAGE");
		ROS_ERROR("ROS ERROR MESSAGE");
		ROS_FATAL("ROS FATAL MESSAGE");
		ROS_INFO("-------------------------");
		rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
```

在rqt中选择 “Plugins” -> “Logging” -> "Console"

----

### rqt-plot

以图形的形式绘制出乌龟的pose信息。
```shell
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key
```

1. 在rqt中选择 “Plugins” -> "Visualization" -> "Plot"
2. 在 “Topic” 中输入 "/tuetle1/pose" 可以查看pose信息

---

### rqt-bag

录制：
1. 在rqt中选择 “Plugins” -> "Logging" -> "Bag"。
2. 点击红色按钮，选择需要订阅的话题，并设置保存路径。

重放：
1. 在rqt中选择 “Plugins” -> "Logging" -> "Bag"。
2. 点击红色按钮旁边的文件夹，播放。
3. 在蓝色的进度条上右击选择 “Publish” 才能发布。


