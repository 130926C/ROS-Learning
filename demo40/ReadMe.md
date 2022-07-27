## demo 40
仿真环境下的机器人导航（SLAM建图）

在阅读这个demo的时候最好配合 赵虚左 的文章 [ROS入门教程-理论与实践（7.1.1导航模块简介)](https://zhuanlan.zhihu.com/p/363857117) 一起看。

在机器人导航中有以下几个关键部分：
1. 全局地图
2. 自身定位
3. 路径规划
4. 运动控制
5. 环境感知

-------

### 1.全局地图

**SLAM**(simultaneous localization and mapping)能够生成增量地图，当机器人从一个未知环中的未知位置开始移动时，在移动过程中根据自身运动行为和位置估计对地图进行构建。生成的地图需要保存下来给后续的任务中使用，在ROS中保存地图的功能包是 map_serve。

在ROS中有很多种方法实现 SLAM，如 gmapping、hector_slam、cartographer、rgbdslam、ORB_SLAM 等。

【注意】SLAM 是一种解决思想，并不是某一个特殊的算法，实现 SLAM 的方法才是算法。另一方面，**SLAM 是机器人导航的重要技术之一，二者并不等价，确切的讲，SLAM 只是实现地图构建和即时定位。**

------

### 2.自身定位

在导航过程的初始化阶段机器人需要知道自身的位置，有GPS信号的地方可以直接使用GPS信号，但没有信号的时候就可以使用SLAM，ROS也提供了一个用于定位的功能包 **amcl** (adaptiveMonteCarloLocalization)自适应的蒙特卡洛定位，使用粒子滤波根已知的地图信息来跟踪机器人姿态。

-----

### 3.路径规划

路径规划分为全局路径规划和本地实时路径规划，ROS提供了 move_base 包来实现路径规划。

1. 全局路径规划：使用Dijstra或者A*算法设计路径最优解。
2. 本地实时规划：使用某些算法来实现避障。

----

### 4.运动控制

在ROS中可以通过发布和订阅某个话题，如 /cmd_vel 来实现传递运动控制指令。

-----

### 5.环境感知

环境感知为上面四个部分提供了信息支持，使用摄像头、激光雷达、编码器等硬件可以提供全局地图、自身定位、路径规划、运动控制过程中所需要的环境信息。


-----

## 导航中的坐标系

对于一个完全陌生的环境而言，机器人就需要根据自身的情况再逆向推理参考系原点并计算坐标系的相对关系，通常有两种方式：

1. 里程计定位：实时收集机器人的 **速度信息** 并发布机器人当前坐标系和父级坐标系的相对关系；（用自己前进速度和时间进行估算）
2. 传感器定位：使用传感器收集外界环境信息并通过匹配计算，发布机器人坐标系和父级坐标系之间的相对关系；（每走一步测一次，每次运动都是根据新测量到的数据）

两种定位方式的优缺点都是互补的，通常是两种结合使用。

||优点|缺点|常用话题名|
|---|---|---|---|
|里程计|定位信息是连续的，没有离散的跳跃|存在累计误差，不利于长距离或长期定位|odom|
|传感器|比里程计更准确|会出现跳变的情况，当标志物较少的情况下精度下降|map|

【注意】在ROS中坐标系层级关系是单继承的，按道理map和odom应该是同级坐标系，但ROS为了让他们满足这一关系默认将map作为odom的父级，应为map的数据要比odom精确。

--------

### 仿真环境需求
1. gmapping 功能包:
```shell
$ sudo apt-get install ros-melodic-gmapping
```
2. 地图服务 功能包：
```shell
$ sudo apt-get install ros-melodic-map-server
```
3. navigation 功能包：
```shell
$ sudo apt-get install ros-melodic-navigation
```

在新建package的时候需要添加以下依赖：
```txt
gmapping map_server amcl move_base
```

------

### gmapping
gmapping 是ROS中比较成熟的SLAM算法之一，可以根据机器人里程计数据和激光雷达数据绘制二维的栅格地图。这也对机器人提出了一定要求：
1. 机器人可以发布里程计消息；
2. 机器人可以发布激光雷达消息；

gmapping 提供了非常详尽的参数列表可以用来设置，参考官网信息 [gmapping](http://wiki.ros.org/gmapping)。

坐标系变化关系的发布者：

* 雷达->基坐标系：robot_state_publisher 或 static_transform_publisher
* 基坐标系->里程计：里程计节点发布

编写一个 nav_slam.launch 文件，内容可以从 gmapping [官方的示例](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/launch/slam_gmapping_pr2.launch) 中粘贴并修改。
```xml
<launch>
    <!-- 是否使用仿真 -->
    <param name="use_sim_time" value="true"/>
    <!-- gmapping 节点 -->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        <!-- 设置雷达话题 -->
        <remap from="scan" to="scan"/>
        <!-- 地图更新时间 -->
        <param name="map_update_interval" value="5.0"/>
        <!-- 雷达最大长度 -->
        <param name="maxUrange" value="16.0"/>

        <!-- 关键的参数 -->
        <!-- 1.基坐标系 -->
        <param name="base_frame" value="footprint" />
        <!-- 2.里程计坐标系 -->
        <param name="odom_frame" value="odom" />
        <!-- 3.传感器坐标系 -->
        <param name="map_frame" value="map" />

        <param name="sigma" value="0.05"/>
        <param name="kernelSize" value="1"/>
        <param name="lstep" value="0.05"/>
        <param name="astep" value="0.05"/>
        <param name="iterations" value="5"/>
        <param name="lsigma" value="0.075"/>
        <param name="ogain" value="3.0"/>
        <param name="lskip" value="0"/>
        <param name="srr" value="0.1"/>
        <param name="srt" value="0.2"/>
        <param name="str" value="0.1"/>
        <param name="stt" value="0.2"/>
        <param name="linearUpdate" value="1.0"/>
        <param name="angularUpdate" value="0.5"/>
        <param name="temporalUpdate" value="3.0"/>
        <param name="resampleThreshold" value="0.5"/>
        <param name="particles" value="30"/>
        <param name="xmin" value="-50.0"/>
        <param name="ymin" value="-50.0"/>
        <param name="xmax" value="50.0"/>
        <param name="ymax" value="50.0"/>
        <param name="delta" value="0.05"/>
        <param name="llsamplerange" value="0.01"/>
        <param name="llsamplestep" value="0.01"/>
        <param name="lasamplerange" value="0.005"/>
        <param name="lasamplestep" value="0.005"/>
    </node>

    <!-- 坐标系转换关系发布者 -->
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />

    <!-- Rviz 节点-->
    <node pkg="rviz" type="rviz" name="rviz_node" />

</launch>
```

下面的小车机器人和gazebo环境代码可以直接从 demo39 中复制。

【注意】这次复制仅复制 launch 文件夹下的 car.launch，整个world和整个urdf文件夹；launch文件夹下的 rviz_sensor.launch 不用复制。

1. 启动 gazebo 仿真环境；
```shell
$ roslaunch demo car.launch
```
2. 启动上面写的 launch 文件；
```shell
$ roslaunch demo nav_slam.launch
```
3. 启动键盘控制节点；
```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
【注意】这里不能再继续启动roscore节点了，因为gazebo启动的时候已经将roscore代起来了，如果启动了会有以下报错：
```shell
$ roscore 
    ... logging to /home/gaohao/.ros/log/36edff50-07f9-11ed-b309-d85ed3aae81f/roslaunch-GPUServer-22223.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://GPUServer:43433/
    ros_comm version 1.14.13


    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: melodic
    * /rosversion: 1.14.13

    NODES

    RLException: roscore cannot run as another roscore/master is already running. 
    Please kill other roscore/master processes before relaunching.
    The ROS_MASTER_URI is http://GPUServer:11311/
    The traceback for the exception was written to the log file
```

在Rviz中添加机器人模型、雷达、Map、TF。

* “TF”->“Frames”中仅选择 footprint、map、odom这三个坐标系。
* “Map”->“Topic”中选择"/map"。

将当前的 Rviz 配置保存到 config 文件夹中。

-----

### 保存地图

为了保存地图需要安装 ROS 的地图服务：
```shell
$ sudo apt-get install ros-melodic-map-server
```

为了将机器人获取到的地图信息保存到本地磁盘上，需要写一个保存地图的launch文件：

**nav_map_save.launch**

```xml
<launch>
    <arg name="filename" value="$(find demo)/map/nav" />
    <node pkg="map_server" type="map_saver" name="map_save" args="-f $(arg filename)" />
</launch>
```
然后执行这个launch文件，之前启动的gazebo和rviz不用退出。没有报错后就能在 map 包下发现两个被创建的文件：nav.yaml 和 nav.pgm。nav.pgm 就是一张图片信息，即刚才构建的地图，可以直接打开。

**nav.yaml** 地图的描述文件
```yaml
# 地图图片的路径
image: /home/gaohao/Desktop/ROS-Learning/demo40/src/demo/map/nav.pgm
# 地图刻度尺：m/pix -> 一个像素对应在现实生活中多少米
resolution: 0.050000
# 地图的位姿信息，相对于rviz原点的位姿
origin: [-50.000000, -50.000000, 0.000000] # 2D地图上[x,y,偏航角度]
# 是否取反
negate: 0
# 占用阈值
occupied_thresh: 0.65
# 空闲阈值，这两个阈值用来判断地图上某一个像素是否是障碍物
free_thresh: 0.196
```

如果不想新建一个 launch 文件的话也可以使用以下命令保存：
```shell
$ rosrun map_server map_saver -f src/demo/map/nav
    [ INFO] [1658301979.538471726]: Waiting for the map
    [ INFO] [1658301979.783950531, 1986.041000000]: Received a 1984 X 1984 map @ 0.050 m/pix
    [ INFO] [1658301979.783971950, 1986.041000000]: Writing map occupancy data to src/demo/map/nav.pgm
    [ INFO] [1658301979.831503092, 1986.089000000]: Writing map occupancy data to src/demo/map/nav.yaml
    [ INFO] [1658301979.831561379, 1986.089000000]: Done
```
其中，/map/后面的是文件名，因为一次会生成两个文件，所以只需要给出文件前缀即可。

------

### 加载地图

**nav_map_load.launch**
```xml
<launch>
    <!-- 此处的值需要根据生成的配置文件指定 -->
    <arg name="map" default="nav.yaml" />
    <node name="map_server" pkg="map_server" type="map_server" args="$(find demo)/map/$(arg map)"/>
</launch>
```
这个执行后就会在 Rviz 中生成一个 map 话题，让后就可以查看地图信息。

【注意】这个操作的前提是所有东西都关闭了，包括 gazebo、rviz，然后运行这个launch文件后再去启动一个节点。

```shell
$ roslaunch demo nav_map_load.launch
$ rosrun rviz rviz
```

然后在打开的Rviz界面中添加 Map。

【注意】这里有一个天坑，因为直接用launch文件启动的所以会自带一个 roscore 节点，也就是说绝对不能提前启动 roscore，否则在 Map 中是看不到话题的。

可以试着更改 nav.yaml 文件中的数值，然后重新运行 nav_map_load.launch，在 Rviz 中点击 “Reset” 按钮重置地图关系。