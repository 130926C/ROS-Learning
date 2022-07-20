## demo 41

仿真环境下的机器人导航（acml定位）

amcl(Adaptive Monte Carlo Localization) 是用于2D机器人的概率定位，实现了自适应(或KLD采样)的蒙特卡洛定位法，可以根据现有的地图使用粒子滤波器推算出机器人当前的位置。

使用步骤：
1. 编写amcl的launch文件；
2. 编写一个测试专用的launch文件；
3. 执行并测试；

同demo40一样，在创建功能包的时候需要添加以下依赖：
```txt
gmapping map_server amcl move_base
```

然后将demo40中的几个文件夹拷贝进来。

-----

### **Step 1** 创建 amcl.launch 文件

在编写这个文件的时候因为涉及到了很多参数，自己写的话容易出错，因此建议是在amcl功能包中拷贝：
```shell
$ roscd amcl
$ gedit examples/amcl_diff.launch  # 设个是amcl对差速机器人的示例，直接拷贝过来
```

**amcl.launch**
```xml
<launch>
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <!-- Publish scans from best pose at a max of 10 Hz -->
        <param name="odom_model_type" value="diff"/>
        <param name="odom_alpha5" value="0.1"/>
        <param name="transform_tolerance" value="0.2" />
        <param name="gui_publish_rate" value="10.0"/>
        <param name="laser_max_beams" value="30"/>
        <param name="min_particles" value="500"/>
        <param name="max_particles" value="5000"/>
        <param name="kld_err" value="0.05"/>
        <param name="kld_z" value="0.99"/>
        <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/>
        <!-- translation std dev, m -->
        <param name="odom_alpha3" value="0.8"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="laser_z_hit" value="0.5"/>
        <param name="laser_z_short" value="0.05"/>
        <param name="laser_z_max" value="0.05"/>
        <param name="laser_z_rand" value="0.5"/>
        <param name="laser_sigma_hit" value="0.2"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_model_type" value="likelihood_field"/>
        <!-- <param name="laser_model_type" value="beam"/> -->
        <param name="laser_likelihood_max_dist" value="2.0"/>
        <param name="update_min_d" value="0.2"/>
        <param name="update_min_a" value="0.5"/>

        <!-- 设置坐标系：odom、map、机器人基坐标系-->
        <!-- 在此处添加一个base_frame_id = footprint -->
        <param name="odom_frame_id" value="odom"/>
        <param name="base_frame_id" value="footprint"/>

        <param name="resample_interval" value="1"/>
        <param name="transform_tolerance" value="0.1"/>
        <param name="recovery_alpha_slow" value="0.0"/>
        <param name="recovery_alpha_fast" value="0.0"/>
    </node>
</launch>
```

-----
### **Step 2** 编写amcl的测试文件

amcl_test.launch
```xml
<launch>
    <!-- 启动 rviz -->  
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
    <node pkg="rviz" type="rviz" name="rviz_node" />

    <!-- 加载地图服务 -->
    <include file="$(find demo)/launch/nav_map_save.launch" />
 
    <!-- 运行amcl -->
    <include file="$(find demo)/launch/amcl.launch" />

</launch>
```

------

### **Step 3** 启动

1. 启动gazebo仿真环境：
```shell
$ roslaunch demo car.launch
```
2. 启动 amcl_test.launch：
```shell
$ roslaunch demo amcl_test.launch
```
3. 在Rviz中添加各种需要的组件
* RobotModel
* Map -> /map
* PoseArray -> /particlecloud
4. 控制机器人运动
```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
【注意】因为roslaunch会自动拉起来一个roscore，所以这个时候不能再新开一个roscore，否则后面的话题会订阅不到。

