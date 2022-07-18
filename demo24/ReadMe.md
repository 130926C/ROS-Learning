### demo24

小乌龟跟踪

【注】乌龟的默认坐标系是左下角原点，如果直接使用pose信息的话需要人工进行计算，乌龟2要跟踪乌龟1时需要手动计算相对位置。假设乌龟1相对于world坐标系移动了（1，2），但并不意味着相对于乌龟2移动了（1，2），对于乌龟2而言乌龟1的移动需要使用world坐标系进行计算。

因此需要进行多坐标系转化来简化这一操作，借助ros自带的坐标系转化功能，让乌龟1始终在乌龟2的相对坐标系上移动。

需要准备的包如下：
roscpp rospy tf2 tf2_ros geometry_msgs std_msgs tf2_geometry_msgs turtlesim

【注意】如果在创建包的时候忘记添加，可以在后面CMakeLists.txt和package.xml文件中追加。

------

### Launch文件

写一个launch文件来一次性启动多个文件。
```shell
<launch>
<!-- 生成可操控的小乌龟 -->
	<node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="turtle1_key" output="screen" />
	
	<!-- 生成一个小乌龟 -->
	<node pkg="demo" type="new_turtle_node" name="turtle2" output="screen"/>
	
	<!-- 发布两个乌龟相对于世界坐标系的关系 -->
	<node pkg="demo" type="turtle_pub_node" name="turtle1_pub" args="turtle1" output="screen"/>
	<node pkg="demo" type="turtle_pub_node" name="turtle2_pub" args="turtle2" output="screen"/>

</launch>
```

-----

### 创建一个新的乌龟

编写一个创建新乌龟的文件 new_turtle.cpp，用服务-客户的方式发送spawn请求。
```cpp
#include "ros/ros.h"
#include "ros/service_client.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[]){
// 1.初始化节点
	ros::init(argc, argv, "new_turtle");
	// 2.获得句柄
	ros::NodeHandle hd;
	
	// 3.创建一个乌龟的spawn请求信息
	turtlesim::Spawn spawn;
	spawn.request.name = "turtle2";
	spawn.request.x = 1.0;
	spawn.request.y = 2.0;
	spawn.request.theta = 0.5;
	
	// 4.创建一个发送spawn信息的客户端
	ros::ServiceClient client = hd.serviceClient<turtlesim::Spawn>("/spawn");
	// 5.发送信息
	client.waitForExistence();
	bool flag = client.call(spawn);
	if (flag){
		ROS_INFO("Spawn Successed!");
	}else{
		ROS_ERROR("Error: Spawn Faild");
	}
	
	return 0;
}
```

-----

### 发布乌龟坐标系信息

```cpp
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/subscriber.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
```

用一个静态变量保存乌龟名
```cpp
static std::string turtle_name;
```

【主函数】用订阅者方式实现订阅某只乌龟的pose信息
```cpp
int main(int argc, char *argv[]){
	// 1.初始化节点
	ros::init(argc, argv, "tf_node");
	// 2.获得句柄
	ros::NodeHandle hd;
	
	// 3.检查参数合法性
	if (argc != 2){
		ROS_ERROR("Error: param should be one");
		return 1;
	}
	turtle_name = argv[1];
	
	// 4.订阅pose话题
	ros::Subscriber sub = hd.subscribe<const turtlesim::Pose &>(turtle_name+"/pose", 100, doPoseMsgs);
	
	ros::spin();
	
	return 0;
}
```

【回调函数】将订阅到的pose信息进行坐标系变化
```cpp
void doPoseMsgs(const turtlesim::Pose & pose){
// 1.创建一个坐标系
	geometry_msgs::TransformStamped tfs;
	// 2.将pose信息潜入到这个坐标系中
	tfs.header.frame_id = "world";
	tfs.header.stamp = ros::Time(0.0);
	tfs.child_frame_id = turtle_name;
	
	tfs.transform.translation.x = pose.x;
	tfs.transform.translation.y = pose.y;
	tfs.transform.translation.z = 0.0;
	
	// 3.转化四元数
	tf2::Quaternion qtu;
	qtu.setRPY(0, 0, pose.theta);
	tfs.transform.rotation.x = qtu.getX();
	tfs.transform.rotation.y = qtu.getY();
	tfs.transform.rotation.z = qtu.getZ();
	tfs.transform.rotation.w = qtu.getW();
	
	// 4.创建一个坐标系发布者(这里需要做成静态的，否则每都会申请导致 /tf 话题无法持续)
	// 这个发布者必须要先初始化节点之后才能创建
	static tf2_ros::TransformBroadcaster pub;
	
	// 5.将信息发布
	pub.sendTransform(tfs);
}
```
【注】static tf2_ros::TransformBroadcaster pub 不能放在外面，因为需要先获得句柄以后才能让这个对象可用。

------

### 控制乌龟运动

```cpp
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/Twist.h"
```

基本操作
```cpp
// 1.初始化节点
ros::init(argc, argv, "control_node");
// 2.获得句柄
ros::NodeHandle hd;
```

创建一个坐标系订阅者，用于订阅两只乌龟的坐标系信息，
```cpp
tf2_ros::Buffer buffer;    // 创建一个存放信息的缓存
tf2_ros::TransformListener listener(buffer);  // 缓存绑定的是坐标系变化的内容
ros::Rate rate(2);
```

创建一个控制乌龟运动的发布者，用来发布控制乌龟运动的话题信息，因为乌龟的控制信息 /turtle1/cmd_vel的类型是 Twist，所以需要用同类型数据进行发布。
```cpp
ros::Publisher pub = hd.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);  // "turtle2/cmd_vel" 也是可以的
```

【注】以下内容都在while循环中，如下所示
```cpp
while(ros::ok()){
	try{
		...
	}
	catch(const std::exception &e){
		ROS_ERROR("Error: %s", e.what());	
	}
	rate.sleep();
	ros::spinOnce();
}
```

坐标系订阅者进行相对转换，将乌龟1的坐标系转为乌龟2的。这个需要放入循环中，因为要实时进行转换。
```cpp
geometry_msgs::TransformStamped turtle2Toturtle1 = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));  // 从buffer中读出一个内容并转化
ROS_INFO("Father:%s, Son:%s, bias:(%.2f, %.2f, %.2f)",
	turtle2Toturtle1.header.frame_id.c_str(),
	turtle2Toturtle1.child_frame_id.c_str(),
	turtle2Toturtle1.transform.translation.x,
	turtle2Toturtle1.transform.translation.y,
	turtle2Toturtle1.transform.translation.z
);
```

准备一个控制乌龟运动的信息 & 发布
```cpp
geoemetry_msgs::Twist twist;
twist.linear.x = 0.5 * sqrt(pow(turtle2Toturtle1.transform.translation.x, 2) + pow(turtle2Toturtle1.transform.translation.y, 2));
twist.angular.z = 0.5 * atan2(turtle2Toturtle1.transform.translation.y, turtle2Toturtle1.transform.translation.x);
pub.publish(twist);
```

python实现见文件夹中的scripts，但是因为出现了PyKDL包问题，疑似和Anaconda相关，所以在这里不实现。

-----

## 小结

在ROS中，坐标变换是非常重要的部分，TF2是专门用来实现坐标变换的组件，官网建议直接学习TF2，主要有以下几个基本操作：
1. 静态坐标变换广播；官方建议用他们提供的命令行实现。
2. 动态坐标变化广播；
3. 坐标变换监听器；

