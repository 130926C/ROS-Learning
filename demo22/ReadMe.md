### demo22

动态坐标变化

动态坐标变化是最常见的坐标变化，但要注意的是，动态坐标变化和静态坐标变化在ROS中用的是完全不同的两套管理方式，因此发布坐标系关系的对象也是不同的，两者不可混用。

动态订阅乌龟的坐标并发布，然后在rviz上展示出来。

---

### 发布方

这里的发布方要完成的其实是两步操作，首先是需要订阅乌龟坐标信息，然后再将这些信息发布出去。

1. 乌龟位置的话题是 /turtle1/pose
2. 乌龟位置的发布信息类型是 /turtlesim/Pose

需要涵盖的头文件
```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
```

信息的回调函数 doPose
```cpp
void doPose(const turtlesim::Pose::ConstPtr &pose){
	// 1.创建发布对象
	static tf2_ros::TransformBroadcaster pub; // 这样使用回调函数使用的就是同一个对象
	// 2.组织发布数据
	geometry_msgs::TransformStamped ts;
	ts.header.frame_id = "world"; // 要转换成相对坐标系的 id
	ts.header.stamp = ros::Time::now();
	ts.child_frame_id = "turtle1";
	// 坐标系偏移量
	ts.transform.translation.x = pose->x;
	ts.transform.translation.y = pose->y;
	ts.transform.translation.z = 0;
	// 坐标系四元数
	/*
	pose 中没有四元数，但有一个theat是偏航角度，同时乌龟是2D的，那么俯仰和翻滚也是0
	*/
	tf2::Quaternion que;
	que.setRPY(0, 0, pose->theta);
	ts.transform.rotation.w = que.getW();
	ts.transform.rotation.y = que.getY();
	ts.transform.rotation.z = que.getZ();
	ts.transform.rotation.x = que.getX();
	// 3.发布
	pub.sendTransform(ts);
}
```

主函数 main
```cpp
int main(int argc, char *argv[]){
	// 1.初始化节点
	ros::init(argc, argv, "pub_node");
	// 2.获得句柄
	ros::NodeHandle hd;
	// 3.订阅乌龟的信息
	ros::Subscriber sub = hd.subscribe<const turtlesim::Pose::ConstPtr&>("turtle1/pose", 100, doPose);
	
	ros::spin();
	return 0;
}
```

在启动乌龟和控制节点后可以在 rostopic 中找到一个新的话题
```shell
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key

$ rostopic list
	/clicked_point
	/rosout
	/tf
	/turtle1/pose
	...

$ rviz
```

------

### 订阅方

动态坐标变换的订阅方实现和静态的比较相似。

这个文件实现了以下几个作用：
1. 从话题中订阅到了乌龟的坐标系；
2. 生成一个相对于乌龟而言静止的点；
3. 将这个静止的点转化为绝对坐标系的动态点；

头文件
```cpp
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // 必须添加这个头文件，否则会编译报错
```

基本操作
```cpp
// 1.初始化节点
ros::init(argc, argv, "sub_node");

// 2.获得句柄
ros::NodeHandle hd;
```

创建一个订阅者
```cpp
// 3.创建一个buffer
tf2_ros::Buffer buffer;

// 4.创建坐标的listener 并传入 buffer
tf2_ros::TransformListener listener(buffer);
```

转换坐标系
```cpp
// 5.创建一个被转换的坐标点
geometry_msgs::PointStamped ps;
ps.header.frame_id = "turtle1";
ps.header.stamp = ros::Time(0.0); // 这里不能用 now

ps.point.x = 2.0;
ps.point.y = 1.0;
ps.point.z = 0.0;

ros::Rate rate(10);
```

订阅信息（try catch）
```cpp
while(ros::ok()){
	geometry_msgs::PointStamped ps_out;
	try{
		ps_out = buffer.transform(ps, "world"); // 被转换的坐标系是绝对坐标系
		ROS_INFO("Transled point=(%.2f, %.2f, %.2f), relative system=%s",
			ps_out.point.x,
			ps_out.point.y,
			ps_out.point.z,
			
			ps_out.header.frame_id.c_str()
		);
	}catch(const std::exception &e){
		ROS_INFO("Exception Message, %s", e.what());
	}
	rate.sleep();
	ros::spinOnce();
}
```

python 实现在目录的 scripts 中。