## Exercise 04

这个实验用来验证在ROS中静态坐标变化的多点接力方式，因为ROS是通过 TF tree 的方式组织坐标系的，所以每当增加一个节点后，这个 TF tree 就会执行一个树的插入操作，通过这种方式将所有坐标系关系组织起来。



注意事项有以下几点：

* 坐标系信息需要完整填充才能发布否则会报错，详见下面描述；
* 静态坐标系发布和动态坐标系发表底层是两套完全不同的逻辑，即 **StaticTransformBroadcaster** 和 **TransformBroadcaster** 不能乱用；
* 无论是发布什么坐标系，头文件 **"tf2_geometry_msgs/tf2_geometry_msgs.h"** 必须有；
* 与其说ROS发布了一个坐标系，不如说是发布了两个坐标系之间的空间位置关系，所以这个坐标系关系其实是一种转化函数，只不过ROS将函数的参数在发布的时候写进去了；
* 在设计的时候务必要将坐标系发布节点和订阅转化节点分开写，这不仅是从效率角度出发，更是从ROS的回旋要求出发，spin和spinOnce的应用场景存在差异；



在联系过程中发现了一个非常严重的坑点：由于ROS是强数据格式要求，这也是处于在工业应用过程的中的严谨，所以ROS要求坐标系必须填充完整的数据才可以发布，否则将发布失败。



如下面的操作因为没有给坐标系添加旋转信息所以会出现报错：

```cpp
    tf2_ros::StaticTransformBroadcaster pub;
    tf2::Quaternion que;
    que.setRPY(0,0,0);

	geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "tfs1";
    tfs.transform.translation.x = 1000;
	// 缺少了旋转的信息
	// tfs.transform.rotation.w = que.getW();
	pub.sendTransform(tfs);
```

同时，这样添加旋转信息也会报错：

```cpp
	tfs1.transform.rotation.w = 0;
```

因为 que.getW() 的返回值是 inline const tf2Scalar &tf2::Quaternion::getW() const，而不是double，因此即便 translation.x 是一个 double 值也不能直接给他赋值。

--------



**实验结果**

```shell
$ rosrun demo sub_node 
[ INFO] [1657885874.757985333]: "world" passed to lookupTransform argument target_frame does not exist. 
[ INFO] [1657885875.757013349]: static_tf_1 point [27.00, 15.00] in world tf is [1027.00, 15.00]
[ INFO] [1657885875.757105205]: static_tf_2 point [43.00, 35.00] in world tf is [43.00, 35.00]
[ INFO] [1657885875.757139098]: static_tf_2 point [43.00, 35.00] in static_tf_1  is [-957.00, 35.00]

[ INFO] [1657885876.756952827]: static_tf_1 point [36.00, 92.00] in world tf is [1036.00, 92.00]
[ INFO] [1657885876.757048538]: static_tf_2 point [49.00, 21.00] in world tf is [49.00, 21.00]
[ INFO] [1657885876.757143523]: static_tf_2 point [49.00, 21.00] in static_tf_1  is [-951.00, 21.00]
```



