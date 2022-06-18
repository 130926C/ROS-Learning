### demo16

工作空间覆盖

虽然不同的工作空间内部的功能包可能不重名，但不同的工作空间内可能会重名。
```shell
	ros/
		turtlesim
	demo16/
		turtlesim
	demoX/
		turtlesim
```
这个时候运行：
```shell
	$ rosrun turtlesim turtlesim_node
```
输出结果如下：
```shell
	$ rosrun turtlesim turtlesim_node 
		[ INFO] [1650420400.552288643]: This is space X self-made turtlesim hello_turtlesim.cpp
```

在ros中，后刷新的工作空间  /devel/setup.bash  的优先级高。
```shell
	优先级：
		demoX > demo16 > 官方包
```

使用命令行查看ros的功能包路径：
```shell
	$ echo $ROS_PACKAGE_PATH
		/home/lucks/Desktop/code/ROS/demoX/src:/home/lucks/Desktop/code/ROS/demo16/src:/opt/ros/noetic/share:
```
路径中哪个在前面就先调用哪个。


