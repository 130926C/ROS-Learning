## demo 34
使用arbotix控制小车运动，小车模型在rvix中运动，需要提前准备小车的模型文件，这里直接从demo30处拷贝。

在这之前需要安装arbotix功能包，有以下两种方法：

**方法一**：命令行安装
```shell
$ sudo apt-get install ros-melodic-arbotix
```

**方法二**：从源码中编译
1. 从下载arbotix源码包。
2. 将这个包放在工作目录下，即 demo34/ 目录下，和第一个src目录同级。
3. 在demo34目录下执行 source devel/setup.bash，更新。
4. catkin_make 编译不报错即可。

-------

**Step 1** 新建一个 config 文件夹并在该文件夹下创建一个 control.yaml 文件，添加以下内容：
```yaml
controllers: {
  base_controller: {
    type: diff_controller, 
    base_frame_id: footprint,
    base_width: 0.2, 
    ticks_meter: 2000,
    Kp: 12, 
    Kd: 12,
    Ki: 0,
    Ko: 52,
    acc_limit: 1.0
  }
}
```

------

**Step 2** 在 launch 文件夹下新建一个 control.launch 文件

这个文件大体上和 car_launch.launch 文件一致，只需要删减和改动一行即可
```xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/xacro/car.xacro"/>

    <node pkg="rviz" type="rviz" name="rviz" />
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

    <!-- 集成 arbotix 控制节点 & 加载参数-->
    <node pkg="arbotix_python" type="arbotix_driver" name="control_node" output="screen">
        <!-- 加载参数 -->
        <rosparam command="load" file="$(find demo)/config/control.yaml" />
        <!-- 开启仿真 -->
        <param name="sim" value="true" />
    </node>
</launch>
```

-------

**Step 3**：从命令行窗口启动控制rvix
```shell
$ roslaunch demo control.launch
```

-------

**Step 4**：更换参考坐标系
在rviz界面的左侧 “Displays”->“Fixed Frame” 中选择 “odom”。

这个odom是轮速计坐标系，如果看不到机器人的话点击左下角的 “Add” -> "RobotModel" 添加机器人即可

--------

**Step 5**：以话题的方式控制小车运动

查看当前的话题可以发现多出来了很多话题：
```shell
$ rostopic list
    /clicked_point
    /cmd_vel
    /diagnostics
    /initialpose
    /joint_states
    /move_base_simple/goal
    /odom
    /rosout
    /rosout_agg
    /tf
    /tf_static
```
其中 /cmd_vel 话题仍然是控制小车运动的节点，直接对这个话题进行 pub 即可，这些命令仍然可以用Tab键进行补全。设置线速度为1，角速度也为1就可以让小车原地转圈。
```shell
$ rostopic pub -r 5 /cmd_vel geometry_msgs/Twist "linear:
    x: 1.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 1.0" 
```

还是在rviz界面中点击 “Add”->“Odometry”->“Topic”->“/odom”可以查看机器人的运动方向。


