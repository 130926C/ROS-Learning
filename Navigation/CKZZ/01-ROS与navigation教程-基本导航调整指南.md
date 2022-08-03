## ROS与navigation教程-基本导航调整指南

原文链接 [ROS与navigation教程-基本导航调整指南](https://www.ncnynl.com/archives/201708/1882.html)

-------

在调试机器人的之前的做的第一步是检查 **硬件传感器** 是否工作正常，主要包括 <u>距离传感器</u>（激光雷达或相机）、<u>里程计</u>（odom）、定位（GPS）。其中距离传感器和里程计是必须的，GPS在室内环境中基本用不上。

-------

### 距离传感器

主要是在 rviz 上查看是否达到 **预期效果** 和 **发布频率**，特别要注意 **坐标系**关系以及前后左右四个方向是否正确。

------

### 里程计

1. 检查 **角速度**：打开rviz并设置成odom，显示查看激光雷达的扫描点，将激光雷达的topic衰减设置大点（如20秒），然后原地旋转机器人 **一周** ，然后查看扫描出来的点和之前的是否完全匹配，理想状态下误差应该不会超过2度。
2. 检查 **线速度**：打开rviz并设置成odom，将机器人对着墙移动，然后检查多次生成的墙的厚度变化，理想状态喜爱误差应该在几厘米。

如果上面的检查出现了较大的误差，那么可以用下面的链接方式进行标定：
* 线速标定：http://www.ncnynl.com/archives/201701/1217.html
* 线速标定：http://www.ncnynl.com/archives/201707/1812.html
* 角速标定：http://www.ncnynl.com/archives/201701/1218.html
* 角速标定：http://www.ncnynl.com/archives/201707/1813.html

------

### 定位

如果上面的两步检查都通过了的话，那么建图和定位通常都不会很坏。

------

### 代价地图

导航和代价地图的配置在 [Setup and Configuration of the Navigation Stack on a Robot](http://wiki.ros.org/navigation/Tutorials/RobotSetup) 和 [costmap_2d](http://wiki.ros.org/costmap_2d) 文档中有详细说明，以下是设置地图参数的建议：

* 确保每个传感器的发布频率和地图的更新速率一致，否则很容易受到警告。
* 设置合适的 transform_tolerance 迟延，因为从 base_link 转化到 map 存在计算耗时，要容忍系统在tf变化过程中的延迟。
* 在算力较差的机器人上可以考虑关闭 map_update_rate，但是这会导致机器人面对障碍物的反映速度变慢。
* 当全局代价地图很大的时候降低 publish_frequency。
* 当机器人只在平面工作的时候用 costmap 模型，如果无法简化到平面则需要使用 voxel_grid 模型。
* 设置局部代价地图和全局代价地图最简单的方法是直接将 local_costmap_params.yaml 覆盖 global_costmap_params.yaml，再将地图的宽高比设置为 1:10 即可。
* 根据机器人尺寸来设置地图分辨率。
* 用rviz检查代价地图质量。

-------

### 局部路径规划器

对于正常加速度限制的机器人使用：dwa_local_planner

对于较低加速度限制的机器人使用：base_local_planner

也可以对导航包添加动态配置：dynamic_reconfigure

以下是关于局部路径规划器的配置建议：

* 设置正确的加速度参数。
* 如果机器人有最低加速度限制，则使用 base_local_planner 并将 dwa 设置为 false，同时将 vx_samples 设置为 8～15 之间，这样可以生成非圆形的曲线。
* 如果amcl效果不好，则设置较高的目标公差 xy_goal_tolerance 和 yaw_goal_tolerance。
* sim_time 的值会对机器人行为造成很大的影响，通常为 1～2 s之间，较大的值可以计算出更平滑的轨迹。

更详细的导航配置可以参加郑开宇的指南 [ROS Navigation Tuning Guide](https://kaiyuzheng.me/documents/navguide.pdf)