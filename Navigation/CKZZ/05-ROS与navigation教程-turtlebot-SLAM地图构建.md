## ROS与navigation教程-turtlebot-SLAM地图构建

原文链接 [ROS与navigation教程-turtlebot-SLAM地图构建](https://www.ncnynl.com/archives/201708/1895.html)

一个常规的自主导航流程如下：
1. 使用gmapping或者hector构建静态地图；
2. 在静态地图的基础上进行导航；

----

### 构建地图

主机：拉起机器人
```shell
$ roslaunch turtlebot_bringup minimal.launch
```

主机：启动gmapping
```shell
$ roslaunch turtlebot_navigation gmapping_demo.launch
```

主机：保存地图
```shell
$ rosrun map_server map_saver -f /tmp/my_map
```

----

### 在静态地图上自主导航

主机：拉起机器人
```shell
$ roslaunch turtlebot_bringup minimal.launch
```

主机：加载地图
```shell
$ export TURTLEBOT_MAP_FILE=/tmp/my_map.yaml 
```

主机：启动定位
```shell
$ roslaunch turtlebot_navigation amcl_demo.launch
```

-----

### 操作 tips
1. 在rviz中选择 map 为坐标系。
2. 初始化定位：在rviz中选择 “2D Pose Estimate” 来指定机器人的方向，这个方向应该和机器人前进的方向一致。
3. 点击 “2D Nav Goal” 实现自主导航。


