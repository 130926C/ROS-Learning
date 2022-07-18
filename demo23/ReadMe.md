### demo23

多坐标系变换（静态）

现在有一个父级坐标系world，和两个子级坐标系son1, son2；求出son1原点在son2中的坐标；son1中某个点在son2中的坐标。

实现步骤：
1. 发布son1和son2相对于world的坐标信息；
2. 订阅并使用tf工具实现son1和son2的转换；
3. 将坐标点进行转换；

更多更复杂的多坐标系变化可以在ROSExperiment中找到。

----

### 发布方

用launch文件打包发布两个静态坐标系：
```launch
<launch>
	<!-- 发布son1相对于world & son2相对于world的坐标关系 -->
	<node pkg="tf2_ros" type="static_transform_publisher" name="son1" args="5 0 1 0 0 0 /world /son1" output="screen" />
	<node pkg="tf2_ros" type="static_transform_publisher" name="son2" args="10 3 5 0 0 0 /world /son2" output="screen" />
</launch>
```

也可以使用命令行进行发布：
```shell
$ rosrun tf2_ros static_transform_publisher 5 0 1 0 0 0 /world /son1
$ rosrun tf2_ros static_transform_publisher 10 3 5 0 0 0 /world /son2
```

-----

### 订阅方

头文件：
```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // 这个必须包含，否则编译报错
#include "geometry_msgs/TransformStamped.h"
```

基本操作：
```cpp
// 1.初始化节点
ros::init(argc, argv, "sub_node");
// 2.获得句柄
ros::NodeHandle hd;
```

创建订阅对象：
```cpp
// 3.创建订阅对象
tf2_ros::Buffer buffer;
tf2_ros::TransformListener listener(buffer);
```

准备一个在son1上的坐标点：
```cpp
geometry_msgs::PointStamped psAtSon1;
psAtSon1.header.stamp = ros::Time::now();
psAtSon1.header.frame_id = "son1";
psAtSon1.point.x = 1;
psAtSon1.point.y = 2;
psAtSon1.point.z = 3;
```

订阅信息：

lookupTransform 函数，这个函数能够借助中间坐标系实现两个坐标系之间的相对位置关系转化。  
因为son1和son2的父级坐标系都是world，因此可以使用这个函数实现转化。
```cpp
ros::Rate rate(10);
while(ros::ok()){
	try{
		// 1.计算son1坐标系相对于son2坐标系而言偏移了多少
		//   这部分是本次demo新增的
		geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("son2", "son1", ros::Time(0));
		ROS_INFO("Son1 relative Son2 msgs: [Father:%s, Son:%s], bias value (%.2f, %.2f, %.2f)",
			son1Toson2.header.frame_id.c_str(),
			son1Toson2.child_frame_id.c_str(),
			son1Toson2.transform.translation.x,
			son1Toson2.transform.translation.y,
			son1Toson2.transform.translation.z
		);

		// 2.计算son1坐标系中某个点在son2坐标系上的位置
		//   这部分就是静态坐标转换
		geometry_msgs::PointStamped psAtSon2 = buffer.transform(psAtSon1, "son2");
		ROS_INFO("point in Son1 is (%.2f, %.2f, %.2f); point in Son2 is (%.2f, %.2f, %.2f), relative system is: %s",
			psAtSon1.point.x, 
			psAtSon1.point.y,
			psAtSon1.point.z,
				
			psAtSon2.point.x,
			psAtSon2.point.y,
			psAtSon2.point.z,
			psAtSon2.header.frame_id.c_str()
		};
	}catch(const std::exception &e){
		ROS_INFO("Error Msgs: %s", e.what());
	}

	// 这里需要注意，在循环体内部只能使用 spinOnce
	ros::spinOnce();
	rate.sleep();
}
```


python 实现在scripts文件夹下

------

使用tf2_tools工具查看坐标系发布关系
```shell
$ sudo apt install ros-noetic-tf2-tools
```

在进入工作目录下使用如下命令生成pdf文件
```shell
$ rosrun tf2_tools view_frames.py
```
在哪个路径调用就会生成在哪里


下面这个链接对如何使用spin和spinOnce有一个简单的介绍：
https://blog.csdn.net/Kevin_Xie86/article/details/106207103

```txt
ros::spin()函数一般不会出现在循环中，因为程序执行到spin()后就不调用其他语句了，也就是说该循环没有任何意义，还有就是spin()函数后面一定不能有其他语句(return 0 除外)，有也是白搭，不会执行的。ros::spinOnce()的用法相对来说很灵活，但往往需要考虑调用消息的时机，调用频率，以及消息池的大小，这些都要根据现实情况协调好，不然会造成数据丢包或者延迟的错误。
```

