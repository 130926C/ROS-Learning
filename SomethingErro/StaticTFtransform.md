## Static TF transform 

这个文件记录了在使用静态坐标变化的过程中可能遇到的错误。

-----

* **“XXX” passed to lookupTransform argument target_frame does not exist.**

这个错误是非常常见的，意思是坐标系找不到，这类报错是因为坐标系没有被广播出去。通常是因为没有把坐标系信息填充完全就 sendTransform 了，如：

```cpp
// 创建发布者
tf2_ros::StaticTransformBroadcaster broadcaster;
// 创建相对坐标系
geometry_msgs::TransformStamped ts;
ts.header.seq = 100;
ts.header.stamp = ros::Time::now();
ts.header.frame_id = "world";		// 参考坐标系名
ts.child_frame_id = "laser";		// 相对坐标系名
// 坐标系laser相对于world的偏移量
ts.transform.translation.x = 0.2;
ts.transform.translation.y = 0.0;
ts.transform.translation.z = 0.5;

// 发布两个坐标系关系
broadcaster.sendTransform(ts);
```

这显然没有将四元数信息附加上去，由于ROS是给机器人使用的一个库，在生产过程中必须非常严谨，如果不制定成员变量也不会自动赋予常量，因此必须要将对象的信息完整填充才能发布：

```cpp
tf2_ros::StaticTransformBroadcaster broadcaster;
geometry_msgs::TransformStamped ts;

ts.header.seq = 100;
ts.header.stamp = ros::Time::now();
ts.header.frame_id = "world";
ts.child_frame_id = "laser";

ts.transform.translation.x = 0.2;
ts.transform.translation.y = 0.0;
ts.transform.translation.z = 0.5;

// 给坐标系laser补全角度信息
tf2::Quaternion qtn;
qtn.setRPY(0,0,0);
ts.transform.rotation.x = qtn.getX();
ts.transform.rotation.y = qtn.getY();
ts.transform.rotation.z = qtn.getZ();
ts.transform.rotation.w = qtn.getW();

broadcaster.sendTransform(ts);
```

当然，ROS也不是真的就这么死板，如果实在不想写这么多东西可以简化成如下形式：

```cpp
tf2::Quaternion que;
que.setRPY(0,0,0);

tfs.transform.translation.x = 100;
tfs.transform.rotation.w = que.getW();
```

【注意】这里绝对不能用 **tfs.transform.rotation.w = 0.0;** 这种写法，否则也会报错.

看上去w的值是一个 double 类型的数据，但去看 que.getW() 可以发现他的返回值是 **inline const tf2Scalar &tf2::Quaternion::getW() const**，因此不能直接给0值。

--------



