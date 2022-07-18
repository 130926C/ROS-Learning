### demo 25

### 命令行方式实现 rosbag

rosbag录制操作并回放。

使用rosbag回放的过程是将执行过的命令再执行一次，之前执行的仍会被保留。

1. 新建一个目录用来存放rosbag的回放信息。
2. 使用如下命令开始录制：
```shell
$ rosbag record -a -o 目标文件夹/目标文件名.bags
```
3. 使用如下命令查看bag中的信息：
```shell
$ rosbag info 目标文件夹/目标文件名.bags
```
4. 【可选】让节点重新按照保存的信息运动：
```shell
$ rosbag play 目标文件夹/目标文件名.bags
```


示例：
```shell
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key

$ mkdir bags
$ rosbag record -a -o bags/turtle_test.bag

$ Ctel + C 结束录制
```

然后会在bags文件夹下生产一个文件：
turtle_test_2022-05-22-11-48-56.bag

```shell
$ rosbag info bags/turtle_test_2022-05-22-11-48-56.bag

path:        bags/turtle_test_2022-05-22-11-48-56.bag
version:     2.0
duration:    23.6s
start:       May 22 2022 11:48:56.37 (1653191336.37)
end:         May 22 2022 11:49:19.93 (1653191359.93)
size:        217.6 KB
messages:    3009
compression: none [1/1 chunks]
types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
             rosgraph_msgs/Log   [acffd30cd6b6de30f120938c17c593fb]
             turtlesim/Color     [353891e354491c51aabe32df673fb446]
             turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
topics:      /rosout                    3 msgs    : rosgraph_msgs/Log  
             /turtle1/cmd_vel          88 msgs    : geometry_msgs/Twist
             /turtle1/color_sensor   1459 msgs    : turtlesim/Color    
             /turtle1/pose           1459 msgs    : turtlesim/Pose
```

因为bag中保存的是各种话题的发布信息和内容，在当前示例中放的是/turtle/cmd_vel信息，所以可以让节点按照这个发布的顺序重新做一次操作。
```shell
$ rosbag play bags/turtle_test_2022-05-22-11-48-56.bag
```

【注】此时并不会新建一个节点，二是在原有运动完后的基础上再按照之前输入键盘的命令顺序运动一次。之前在输入过程中如果有停顿也会被录制进来。

当然，因为rosbag本质上也是一个节点，仍然可以用rosrun的方式启动：
```shell
$ rosrun rosbag play bags/turtle_test_2022-05-22-11-48-56.bag
```


------

### rosbag写自定义话题和消息

头文件
```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
```

基本操作
```cpp
// 1.初始化节点
ros::init(argc, argv, "bag_node");
// 2.获得句柄
ros::NodeHandle hd;
```

创建bag对象
```cpp
rosbag::Bag bag;
```

打开文件流
```cpp
bag.open("/home/lucks/Desktop/code/ROS/demo25/src/demo/bags/test.bag", rosbag::BagMode::Write);
```
【注】此处给的路径最好是绝对路径，如果是相对路径的话会在执行路径下进行保存。

准备数据
```cpp
std_msgs::String msg;
msg.data = "hello, this is rosbag";
```

写数据
```cpp
bag.write("/chatter", ros::Time::now(), msg);
```
这个重载函数有三个参数：
* 参数1：话题名
* 参数2：写入消息的时间戳
* 参数3：写入的消息

关闭文件
```cpp
bag.close();
```
【注】一定要养成良好的习惯将文件关闭。

------

### rosbag读话题和消息

头文件：
```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
```

基本操作：
```cpp
// 1.初始化节点
ros::init(argc, argv, "read_node");
// 2.获得句柄
ros::NodeHandle hd;
```

创建bag对象：
```cpp
rosbag::Bag bag;
```

打开文件：
```cpp
bag.open("/home/lucks/Desktop/code/ROS/demo25/src/demo/bags/test.bag", rosbag::BagMode::Read);
```

循环读取数据：
```cpp
// 此处也可以写成 for(auto &&m: rosbag::View(msg))
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
```
这里rosbag提供了一个View的方法用来读取数据，返回一个有迭代器的对象。

