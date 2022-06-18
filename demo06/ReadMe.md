### demo06
控制小乌龟自行运动，官方案例中的小乌龟是通过rostopic实现的，从这个点切入。

Step 1: 首先需要获得控制小乌龟节点的信息
```shell
$ roscore
$ rosrun turtlesim
$ rosrun turrle_teleop_key
```

Step 2: 使用节点图获取活动的节点和使用的话题
```shell
$ rqt_graph     
# 得到节点信息图，/teleop_turtle节点通过话题 turtle1/cmd_vel 将信息传递给 /turtlesim节点
```

Step 3:  查看 turtle1/cmd_vel 的数据类型，可以知道他的 Type: geometry_msgs/Twis
```shell
$ rostopic info /turtle1/cmd_vel
```

Step 4: 查看 geometry_msgs/Twis 类型的内容
```shell
$ rosmsg info geomerty_msgs/Twis
	geometry_msgs/Vector3 linear
	  float64 x
	  float64 y
	  float64 z
	geometry_msgs/Vector3 angular
	  float64 x
	  float64 y
	  float64 z
```

至此，我们可以知道控制小乌龟运动的是这三个参数，可以分别改下试试效果。如果想要其圆周运动的话是修改 linear.x 和 angular.z。 使用发布者方式对这两个数据进行修改，需要包含 geometry_msgs 这个功能包。

Step 5: 也可以直接通过命令行进行修，其中 '-r 1' 指的是 ros::Rate rate(1)
```shell
$ rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist "
	linear:
	  x: 2.0
	  y: 0.0
	  z: 0.0
	angular:
	  x: 0.0
	  y: 0.0
	  z: 1.0" 
```
