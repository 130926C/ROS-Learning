## demo 43

仿真环境下的机器人导航（自建地图与路径规划）

在之前的demo中机器人地图是来自于键盘鼠标控制生成的地图，需要前期得到地图后运行，但SLAM本质是实时地图构建与路径规划，SLAM在运行的本身就会发布一个话题名为 /map 的地图信息，因此无需再使用 map_server 节点。

----

### **Step 1**：集成 SLAM 和 路径规划
**slam_auto.launch**
```xml
<launch>
    <!-- SLAM 实现 -->
    <include file="$(find demo)/launch/nav_slam.launch" />

    <!-- move_base 实现 -->
    <include file="$(find demo)/launch/move_base_path.launch" />

</launch>
```

----

### **Step 2**：运行
1. 启动gazebo仿真环境
```shell
$ roslaunch demo car.launch
```
2. 启动 Rviz
```shell
$ roslaunch demo slam_auto.launch
```
3. 保存地图
```shell
$ roslaunch demo nav_map_save.launch
```

-----

### **关于深度相机转激光雷达**

深度相机成本一般会低于激光雷达，但检测范围和检测精度会比激光雷达差很多，SLAM的效果往往没有激光雷达好。

ROS 提供了一种深度相机转激光雷达的方法，需要先安装对应的功能包：
```shell
$ sudo apt-get install ros-melodic-depthimage-to-laserscan
```

转换节点为 depthimage_to_laserscan，这个节点订阅 **sensor_msgs/Image** 话题来获取图像，用 **sensor_msgs/LaserScan** 来发布激光雷达的话题。
 
------

### 有关地图信息的数据：

* nav_msgs/MapMetaData 地图元数据（地图宽高分辨率）
```shell
$ rosmsg info nav_msgs/MapMetaData 
    time map_load_time      # 加载时间
    float32 resolution      # 分辨率
    uint32 width
    uint32 height
    geometry_msgs/Pose origin   # 地图位姿（偏移量、四元数）
    geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w

```
* nav_msgs/OccupancyGrid 地图栅格数据
```shell
$ rosmsg info nav_msgs/OccupancyGrid 
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    nav_msgs/MapMetaData info   # 这部分其实就是地图元数据
    time map_load_time
    float32 resolution
    uint32 width
    uint32 height
    geometry_msgs/Pose origin
        geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
        geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    int8[] data         # 地图内容数据，数组长度=width*height
```

* /map 地图服务相关的话题，地图信息都会被发布到这个话题上
```shell
$ rostopic info /map
    Type: nav_msgs/OccupancyGrid

    Publishers: 
    * /slam_gmapping (http://GPUServer:40909/)

    Subscribers: 
    * /move_base_node (http://GPUServer:45957/)
    * /rviz_node (http://GPUServer:41013/)
```

* /odom 里程计
```shell
$ rostopic info /odom 
    Type: nav_msgs/Odometry

    Publishers: 
    * /gazebo (http://GPUServer:41623/)

    Subscribers: 
    * /move_base_node (http://GPUServer:45957/)
    * /rviz_node (http://GPUServer:41013/)
```

* /move_base/goal 导航目标点
```shell
$ rostopic info /move_base/goal 
    Type: move_base_msgs/MoveBaseActionGoal

    Publishers: 
    * /move_base_node (http://GPUServer:45957/)

    Subscribers: 
    * /move_base_node (http://GPUServer:45957/)
```

* nav_msgs/Path 路径规划数据
```
$ rosmsgs info nav_msgs/Path
```

* sensor_msgs/LaserScan 激光雷达数据
```shell
$ rosmsg info sensor_msgs/LaserScan 
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    float32 angle_min           # 激光雷达的角度范围min
    float32 angle_max           # 激光雷达的角度范围max
    float32 angle_increment     # 两束激光之间的夹角
    float32 time_increment      # 一道激光的测量时间
    float32 scan_time           # 每次扫描的间隔时间
    float32 range_min           # 扫描最小有效距离
    float32 range_max           # 扫描最大有效距离
    float32[] ranges            # 一次扫描得到的数据
    float32[] intensities       # 扫描强度，如果设备不支持则设置成空
```

* sensor_msgs/Image 原始图像数据
```shell
$ rosmsg info sensor_msgs/Image 
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    uint32 height
    uint32 width
    string encoding     # 编码格式
    uint8 is_bigendian  # 存储大小端
    uint32 step         # 一行图像存储的字节数，步进参数
    uint8[] data        # size=step * height
```
对于图像数据而言，虽然存在深度相机和rgb相机不同的类型，但其实发布的数据类型是一样的：
```shell
$ rostopic info /camera/image_raw
$ rostopic info /depth_camera/rgb/image_raw
$ rostopic info /depth_camera/depth/image_raw

    Type: sensor_msgs/Image
```

* /depth_camera/depth/points 点云数据
```shell
$ rostopic info /depth_camera/depth/points 
    Type: sensor_msgs/PointCloud2

    Publishers: 
    * /gazebo (http://GPUServer:41623/)

    Subscribers: None
```