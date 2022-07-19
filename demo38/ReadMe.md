## demo 38

URDF、Gazebo、Rviz综合使用。

* URDF：创建机器人模型；
* Rviz：用来显示机器人接受到的环境信息（如里程计）；
* Gazebo：用来进行机器人仿真，给机器人输入信息；

在后期可能会需要大量的配置文件，这个连接 https://classic.gazebosim.org/tutorials?tut=ros_gzplugins 可以拿到现成的配置文件。

这些配置文件可以用来模拟深度相机、雷达、IMU等硬件。

------

ros_control 提供了一种接口的 **概念** ，只要按照接口的设计规范使用就可以让代码在多个不同的平台上运行。

运动控制实现：
1. 创建完成机器人后单独编写一个xacro文件，为机器人模型添加传动装置以及控制器；
2. 将文件集成进xacro中；
3. 启动Gazebo并向 /cmd_vel 发布消息；

此处的小车文件、launch文件、world文件都可以直接复制 demo36 的。

-------


### **Step 1**:将运动控制文件集成进car.xacro
**car.xacro**
```xml
<robot name="car" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="head.xacro" />

    <xacro:include filename="car_base.xacro" />
    <xacro:include filename="car_camera.xacro" />
    <xacro:include filename="car_lader.xacro" />
    
    <!-- 将运动控制的 move.xacro 文件集成进来 -->
    <xacro:include filename="gazebo/move.xacro" />
    
</robot>
```

-----

### **Step 2**:编写move.xacro

**move.xacro**

这里可以直接从 赵老师 的知乎专栏中复制 [ROS入门教程-理论与实践（6.7.1 机器人运动控制以及里程计信息显示](https://zhuanlan.zhihu.com/p/362750088)。里面的大部分内容也是从 [官方文档](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)中 **Differential Drive 差速驱动** 部分复制下来的。

【注意】：
1. 这里有大量的地方需要修改，一定要按照自己的实际文件进行改动，在此demo中是 car_base.xacro，因为主要的运动关系都在这个文件中描述；
2. 这个示例仅适用于 **两轮** 的机器人；
```xml
<robot name="my_car_move" xmlns:xacro="http://wiki.ros.org/xacro">

    <!-- 传动实现:用于连接控制器与关节 -->
    <xacro:macro name="joint_trans" params="joint_name">
        <!-- Transmission is important to link the joints and the controller -->
        <transmission name="${joint_name}_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="${joint_name}">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="${joint_name}_motor">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    </xacro:macro>

    <!-- 每一个驱动轮都需要配置传动装置 -->
    <!-- 【注意】需要修改的地方是这里，意思是传动关节的名称 -->
    <!-- 在car_base.xacro 中找到驱动轮的 joint 宏 ${wheel_name}_joint_base-->
    <!-- 将这块写死，${wheel_name} 就用下面的调用 xacro:drive_wheel_func 中的 name -->
    <xacro:joint_trans joint_name="left_joint_base" />
    <xacro:joint_trans joint_name="right_joint_base" />

    <!-- 控制器 -->
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <rosDebugLevel>Debug</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <robotNamespace>/</robotNamespace>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <alwaysOn>true</alwaysOn>
            <updateRate>100.0</updateRate>
            <legacyMode>true</legacyMode>
            <!-- 这里设置关节名称 -->
            <leftJoint>left_joint_base</leftJoint> <!-- 左轮 -->
            <rightJoint>right_joint_base</rightJoint> <!-- 右轮 -->
            <!-- 这里设置车轮间距 : 车体半径 * 2-->
            <wheelSeparation>${car_base_radius * 2}</wheelSeparation> <!-- 车轮间距 -->
            <!-- 这里设置车轮直径 -->
            <wheelDiameter>${drive_wheel_radius * 2}</wheelDiameter> <!-- 车轮直径 -->
            <broadcastTF>1</broadcastTF>
            <wheelTorque>30</wheelTorque>
            <wheelAcceleration>1.8</wheelAcceleration>
            <!-- 控制运动的话题名 -->
            <commandTopic>cmd_vel</commandTopic> <!-- 运动控制话题 -->
            <!-- 里程计的坐标系和订阅话题 -->
            <odometryFrame>odom</odometryFrame> 
            <odometryTopic>odom</odometryTopic> <!-- 里程计话题 -->
            <!-- 机器人模型的根坐标系 -->
            <robotBaseFrame>footprint</robotBaseFrame> <!-- 根坐标系 -->
        </plugin>
    </gazebo>

</robot>
```

-----

### **Step 3**: 在gazebo中启动
```shell
$ roscore
$ source devel/setup.bash
$ roslaunch demo car.launch
```
此时可以查看ros的话题信息：
```shell
$ rostopic list
    /clock
    /cmd_vel
    /gazebo/link_states
    /gazebo/model_states
    /gazebo/parameter_descriptions
    /gazebo/parameter_updates
    /gazebo/set_link_state
    /gazebo/set_model_state
    /joint_states
    /odom
    /rosout
    /rosout_agg
    /tf
```
因为在 move.xacro 中 \<commandTopic\> 标签设置的是 cmd_vel 所以可以使用键盘来控制机器人运动，直接复用小乌龟的运动控制节点就可以。

这里需要用到 ros 的 teleop_twist_keyboard 包，只要你在打字的时候无法自动补齐就说明没有安装上，直接用命令行安装：
```shell
$ sudo apt-get install ros-melodic-teleop-twist-keyboard
```
安装完成之后就可以执行下面的命令调出键盘控制节点：

【注意】即便是安装完了之后也不一定能补齐，如果还是无法补齐的话先把前面的 **teleop_twist_keyboard** 敲上去，然后Tab补齐后面的就可以。
```shell
$ rosrun  teleop_twist_keyboard teleop_twist_keyboard.py 
```

节点启动后默认的线速度和角速度为：
```shell
$ currently:	speed 0.5	turn 1.0
```
可以在节点启动的时候修改线速度和角速度：
```shell
$ rosrun  teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.3 _tun:0.5
```

------

### **Step 4**: 使用Rviz查看里程计odom输出信息
由于gazebo无法发布里程计信息，所以只能使用rviz来查看，因此需要写启动rviz的rviz_sensor.launch文件。

在这个launch文件中需要发布关节状态信息。
```xml
<launch>
    <node pkg="rviz" type="rviz" name="rviz" />

    <!-- 关节以及机器人状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

</launch>
```

【注意】在视频中的 node 标签后面跟了 args 参数，实际上并不需要这个参数；

在启动rviz的launch文件之前首先确保已经启动了gazebo仿真节点。
```shell
$ roslaunch demo rviz_sensor.launch
```

1. “Add” -> "RobotModel" 
2. "Global Options" -> "Fixed Frame" -> "odom"
3. "Add" -> "Odometry"
4. "Odometry" -> "Topic" -> "odom"

也可以在Rviz中添加 TF 坐标变化。

上面 Topic 中的 odom 是在 move.xacro 中的 \<odometryFrame\> 标签定义的，可以自己改变这个话题名，但通常情况下不宜更改。

【注意】如果在rviz中发现添加了模型后模型报错，请检查gazebo是否正常开启，不仅仅要检查GUI界面，更要检查命令行有没有被kill掉，以及roscore节点是否正常运行。

然后就可以用先前打开的 **teleop_twist_keyboard** 节点操控小车运动，小车的里程计信息也会被显示在Rviz界面上。