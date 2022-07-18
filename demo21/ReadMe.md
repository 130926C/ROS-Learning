### demo21

静态坐标变化 TransForm Frame (TF)

静态坐标指的是相对于某一坐标系而言完全静止，在整个生命周期中都不会发生相对位置和旋转变化的坐标系。

在ROS中坐标信息是通过发布者Pub和订阅者Sub进行交换的。

查看坐标点 PointStamped 消息的格式：
```shell
	$ rosmsg info geometry_msgs/PointStamped 
		std_msgs/Header header
		  uint32 seq
		  time stamp
		  string frame_id
		geometry_msgs/Point point
		  float64 x
		  float64 y
		  float64 z
```

查看坐标系 TransformStamped 消息的格式：
```shell
	$ rosmsg info geometry_msgs/TransformStamped 
		std_msgs/Header header
		  uint32 seq
		  time stamp
		  string frame_id
		string child_frame_id
		geometry_msgs/Transform transform
		  geometry_msgs/Vector3 translation
		    float64 x
		    float64 y
		    float64 z
		  geometry_msgs/Quaternion rotation
		    float64 x
		    float64 y
		    float64 z
		    float64 w
```

translation：当前坐标系相对于零坐标系的空间位置；
rotation：当前坐标系相对于零坐标系的旋转角度；

-------

**发布方实现 Publisher** 

静态的坐标信息只需要发布一次就可以了，因为发布者位置不变。

具体步骤如下：
0. 添加必要的头文件
```cpp
	#include "tf2_ros/static_transform_broadcaster.h"
	#include "geometry_msgs/TransformStamped.h"
	#include "tf2/LinearMath/Quaternion.h"
```
1. 创建一个发布对象
```cpp
	tf2_ros::StaticTransformBroadcaster pub;
```
2. 创建需要发布的信息
```cpp
	geometry_msgs::TransformStamped msg;
```
3. 填写头内容
```cpp
	msg.header.frame_id = "base_line";    // 基准坐标的名字
	msg.header.stamp = ros::Time::now();  // 当前信息的时间戳
	msg.child_frame_id = "rel_t";         // 相对坐标系的名字
```
4. 相对坐标系原点和基准坐标系原点的位置
```cpp
	tfs.transform.translation.x = 3.4;
	tfs.transform.translation.y = 2.1;
	tfs.transform.translation.z = 0.8; 
```
5. 相对坐标系和基准坐标系的相对旋转角度
```cpp
	tf2::Quaternion quate;   // 四元数和欧拉角转换对象
	quate.setRPY(1.25, 3.00, 0.51);  // 填写角度
	
	msg.transform.rotation.x = quate.getX();
	msg.transform.rotation.y = quate.getY();
	msg.transform.rotation.z = quate.getZ();
	msg.transform.rotation.w = quate.getW();
```
使用欧拉角会出现“万象节死锁”，所以ROS的坐标系统就不提供欧拉角的方案，只允许使用四元数方法设置。

【注】即便不想让坐标系有旋转那也需要填写 transform.rotation.x 这些参数，因为发布的数据类型必须完整。让 setRPY(0,0,0) 即可。

6. 发布信息
```cpp
	pub.sendTransform(msg);
	ros::spin();
```

-------

**订阅方实现**

假设现在有一个坐标点是雷达采集到的点，那么目标是将这个点通过发布方转换为零坐标系的位置。

0. 包含头文件
```cpp
	#include "ros/ros.h"
	#include "tf2_ros/transform_listener.h"
	#include "tf2_ros/buffer.h"
	#include "geometry_msgs/PointStamped.h"
	#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
```

1. 创建订阅对象 & 将buffer绑定到订阅对象上
```cpp
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener(buffer);
```

2. 采集（创建）一个 PointStamped 格式的坐标点
```cpp
	geometry_msgs::PointStamped pst;
	pst.header.frame_id = "laser";
	pst.header.stamp = ros::Time::now();

	pst.point.x = 1.0; // 实际场景下这个坐标点是循环更新的，这里写死
	pst.point.y = 2.3;
	pst.point.z = 0.2;
```

3. 休眠一下等待发布方发布消息
```cpp
	ros::Duration(2).sleep();
```

4. 接受发布方的消息，并根据发布消息中的头信息将采集到的坐标点转为绝对坐标
```cpp
	ros::Rate rate(10);
	while(ros::ok()){
		// 4.将雷达采集到的坐标点转换成base_link的位置
		geometry_msgs::PointStamped ps_out;   // 转换后的结果
		ps_out = buffer.transform(pst, "base_link");
		
		ROS_INFO("Transled point=(%.2f, %.2f, %.2f), relative system=%s",
			ps_out.point.x,
			ps_out.point.y,
			ps_out.point.z,
			ps_out.header.frame_id.c_str()
		);
		
		rate.sleep();
		ros::spinOnce();
		}
```
【注】调用转换功能时必须包含头文件 "tf2_geometry_msgs/tf2_geometry_msgs.h"，否则会报错。

4. 如果在上面不进行休眠，则需要进行异常处理
```cpp
	ros::Rate rate(10);
	while(ros::ok()){
	// 4.将雷达采集到的坐标点转换成base_link的位置
		geometry_msgs::PointStamped ps_out; // 转换后的结果
			
		try{
			ps_out = buffer.transform(pst, "base_link");
			ROS_INFO("Transled point=(%.2f, %.2f, %.2f), relative system=%s",
			ps_out.point.x,
			ps_out.point.y,
			ps_out.point.z,
			ps_out.header.frame_id.c_str()
			);
		}
		catch(const std::exception& e){
			ROS_INFO("Exception Message, %s", e.what());
		}
		
		rate.sleep();
		ros::spinOnce();
	}
```
【注】这里的处理逻辑是，捕获异常后不进行处理，只输出一句话，然后让他再次循环，这样就可以订阅到发布者的信息。

-----

命令行中使用图形化方式查看两个坐标系位置关系：
```shell
	$ rviz
```
在启动的界面中执行以下操作：
1. 在Fixed Frame中选择base_link；
2. 在Add中添加TF；

之所以用四元数代替欧拉角作为信息发布，是因为欧拉角会出现“万向节死锁”的情况。因为陀螺仪中存在万向节死锁的问题，所以游戏中如果使用的是欧拉角的旋转方案，需要限制在某个方向的最大旋转角度（枪战无法看到头顶正上方和脚底正下方，说明死锁了俯仰方向x）。

这里所说的死锁并不是无法恢复的意思，而是想要恢复的话需要三个轴同时作用才能达到原先只用一个轴的效果，并且移动的路径也不是弧线而是不规则曲线。

B站上这个教程非常好：
1. https://www.bilibili.com/video/BV1YJ41127qe/?spm_id_from=333.788.recommend_more_video.0
2. https://www.bilibili.com/video/BV1zF411a7j1?spm_id_from=333.337.search-card.all.click

-----

**代码复用（官方建议）**

ros其实已经封装好了静态坐标变化的命令
```shell
	$ rosrun tf2_ros static_transform_publisher rx ry rz ez ey ex base_line child_line
```
rx, ry, rz：子坐标系相对于父坐标系的位置关系
ez, ey, ex：子坐标系相对于父坐标系的欧拉角度变换
base_line：父坐标系名称
child_line：子坐标系名称

