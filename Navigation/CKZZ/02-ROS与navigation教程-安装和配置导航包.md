## ROS与navigation教程-安装和配置导航包

原文链接 [ROS与navigation教程-安装和配置导航包](https://www.ncnynl.com/archives/201708/1883.html)。

这是ROS官方导航功能包的经典配图。
![ROS 官方导航功能包](../images/overview_tf_small.png)

------

### 拉起所有传感器

my_robot_configuration.launch
```xml
<launch>
    <node pkg="传感器ros包名" type="传感器数据节点" name="sensor_node_name" output="screen">
        <param name="传感器可选参数" value="param_value" />
    </node>

    <node pkg="里程计或小车包名" type="节点" name="odom_node" output="screen">
        <param name="可选参数" value="param_value" />
    </node>

    <node pkg="transform_configuration_pkg" type="transform_configuration_type" name="transform_configuration_name" output="screen">
        <param name="transform_configuration_param" value="param_value" />
    </node>
</launch>
```

在上面的代码中，部分小车的底盘SDK是自带里程计的并且在启动底盘节点的时候会直接拉起里程计节点，因此需要看厂商的文档。

------

### 通用配置的代价地图

关于代价地图更详细的配置可以参考官方文档 [costmap_2d](http://wiki.ros.org/costmap_2d)。

costmap_common_params.yaml
```yaml
# 障碍物信息阈值
obstacle_range: 2.5     # 更新半径在2.5m以内的障碍物信息
raytrace_range: 3.0     # 忽略半径在3.0m以外的障碍物信息

# 机器人占用面积与膨胀系数
footprint: [[x0, y0], [x1, y1], ... [xn, yn]]
inflation_radius: 0.55

# 代价地图接受传感器的列表，以空格分割
observation_sources: laser_scan_sensor point_cloud_sensor

# 对每个传感器进行定义。frame_name 如 laser_link 指传感器坐标系名称
laser_scan_sensor: {sensor_frame: frame_name, data_type: LaserScan, topic: topic_name, marking: true, clearing: true}

point_cloud_sensor: {sensor_frame: frame_name, data_type: PointCloud, topic: topic_name, marking: true, clearing: true}
```

global_costmap_params.yaml
```yaml
global_costmap:
  global_frame: /map
  robot_base_frame: base_link
  update_frequency: 5.0
  static_map: true
```

local_costmap_params.yaml
```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  # 机器人在移动时保持在本地代价地图的中心
  rolling_window: true
  width: 6.0
  height: 6.0
  resolution: 0.05
```

------

### 本地规划器配置

base_local_planner_params.yaml

关于规划器更详细的配置可以参考文档 [base_local_planner](http://wiki.ros.org/base_local_planner)

这个文件负责将规划器计算得到的数据发送给机器人基底，并控制其运动。
```yaml
TrajectoryPlannerROS:
  # 速度限制 
  max_vel_x: 0.45
  min_vel_x: 0.1
  max_vel_theta: 1.0
  min_in_place_vel_theta: 0.4

  # 加速度限制
  acc_lim_theta: 3.2
  acc_lim_x: 2.5
  acc_lim_y: 2.5

  holonomic_robot: true
```

------

### 启动导航功能包

move_base.launch

```xml
<launch>
    <master auto="start"/>

    <!-- map server -->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find my_map_package)/my_map.pgm my_map_resolution"/>

    <!--- AMCL -->
    <include file="$(find amcl)/examples/amcl_omni.launch" />

    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <!-- 给全局和本地代价地图配置通用参数 -->
        <rosparam file="$(find my_robot_name_2dnav)/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find my_robot_name_2dnav)/costmap_common_params.yaml" command="load" ns="local_costmap" />

        <!-- 载入本地代价地图配置 -->
        <rosparam file="$(find my_robot_name_2dnav)/local_costmap_params.yaml" command="load" />
        <!-- 载入全局代价地图配置 -->
        <rosparam file="$(find my_robot_name_2dnav)/global_costmap_params.yaml" command="load" />

        <!-- 载入路径规划器配置 -->
        <rosparam file="$(find my_robot_name_2dnav)/base_local_planner_params.yaml" command="load" />
    </node>
</launch>
```

启动机器人
```shell
roslaunch my_robot_name_2dnav my_robot_configuration.launch
```

启动导航
```shell
roslaunch my_robot_name_2dnav move_base.launch
```

