### demo10

参数重名，全局、私有

1. rosrun 方式
2. launch 文件
3. 编码方式

-----

**rosrun**

命令行传参
```shell
	$ rosrun turtlesim turtlesim_node _paramA:=10
```

--------

**launch**

全局：在node外
私有：在node内
```launch
<launch>

	<!--在node外：全局 -->
	
	<param name="radius" type="double" value="0.22" />
	
	<!--在node内：私有 -->
	<node pkg="turtlesim" type="turtlesim_node" name="turtle_node" output="screen">
		<param name="radius" type="double" value="1.32" />
	</node>

</launch>
```

```shell
	$ rosparam list
		/radius
		/turtle_node/radius
```

----

**编码**

方案一：用ros::param方式设置
```cpp
// A.全局
ros::param::set("/param_global", -1);

// B.相对
ros::param::set("param_relative", 500);

// C.私有
ros::param::set("~param_private", 3.14);
```

```shell
	$ rosparam list
		/param_global
		/paran_node/param_private
		/param_relative
```


方案二：用 NodeHandle 方式
```cpp
// A.全局
hd.setParam("/param_global", -1);

// B.相对
hd.setParam("param_relative", 500);

// C.私有
ros::NodeHandle nd("~");
nd.setParam("param_private", 3.14);
```

Python实现：
```python
# 1.全局
rospy.set_param("/param_global", 10)

# 2.相对
rospy.set_param("param_relative", 20)

# 3.私有
rospy.set_param("~param_private", 30)
```


