## demo 39

在 demo38 的基础上给小车添加各种传感器，包括雷达、摄像头、深度相机

1. 单独创建一个xacro文件，为机器人添加雷达模型；
2. 将该文件集成到机器人xacro文件中；
3. 启动Gazebo，使用Rviz显示雷达信息；

------

### **Step 1**：雷达
这部分信息仍然是从官网上可以找到，或者在 赵虚左 的知乎博客 [ROS入门教程-理论与实践（6.7.2 雷达信息仿真以及显示)](https://zhuanlan.zhihu.com/p/363043128) 中找到。

需要改动的关键位置有三处：
1. \<gazebo reference="laser"\> ：雷达link的名字；
2. \<topicName\> ：雷达信息发布到的话题名；
3. \<frameName\> ：雷达link的名字 ；

**laser.xacro**
```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
  <!-- 雷达 -->
  <!-- 这里的reference用的是 car_laser.xacro 中的雷达link -->
    <gazebo reference="laser">
        <!-- 雷达的类型 -->
        <sensor type="ray" name="rplidar">
            <!-- 雷达的偏移量 -->
            <pose>0 0 0 0 0 0</pose>
            <!-- 是否显示雷达射线 -->
            <visualize>true</visualize>
            <!-- 雷达发射频率 -->
            <update_rate>5.5</update_rate>
            <ray>
                <scan>
                  <horizontal>
                      <!-- 雷达旋转一周采样点个数 -->
                      <samples>360</samples>
                      <!-- 雷达分辨率 -->
                      <resolution>1</resolution>
                      <!-- 雷达采样范围（弧度 -PI~PI）-->
                      <min_angle>-3</min_angle>
                      <max_angle>3</max_angle>
                  </horizontal>
                </scan>
                <range>
                  <!-- 有效采样范围 -->
                  <min>0.10</min>
                  <max>30.0</max>
                  <!-- 采样精度 -->
                  <resolution>0.01</resolution>
                </range>
                <noise>
                    <!-- 高斯噪声(模拟误差) -->
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
                <!-- 雷达发布消息到的位置 --> 
                <topicName>/scan</topicName>
                <!-- 这里的名字也必须和 car_laser.xacro 中的雷达link 名字一致 -->
                <frameName>laser</frameName>
            </plugin>
        </sensor>
    </gazebo>
</robot>
```

-------

### **Step 2**：摄像头
这个同雷达一样需要注意的地方有三个：
1. \<gazebo reference="laser"\> ：摄像头link的名字；
2. \<cameraName\> ：摄像头信息发布到的话题名；
3. \<frameName\> ：摄像头link的名字 ；

**camera.xacro**
```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 被引用的link -->
    <!-- 是 car_camera.xacro 中的link名字 -->
    <gazebo reference="camera">
        <!-- 类型设置为 camara -->
        <sensor type="camera" name="camera_node">
            <update_rate>30.0</update_rate> <!-- 更新频率 -->
            <!-- 摄像头基本信息设置 -->
            <camera name="head">
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                <width>1280</width>
                <height>720</height>
                <format>R8G8B8</format>
                </image>
                <clip>
                <near>0.02</near>
                <far>300</far>
                </clip>
                <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
                </noise>
            </camera>
            <!-- 核心插件 -->
            <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
                <alwaysOn>true</alwaysOn>
                <updateRate>0.0</updateRate>
                <!-- 摄像头发布的话题名 -->
                <!-- 此处有一个命名空间，最终的toplic是 /camera/image_raw -->
                <cameraName>/camera</cameraName>
                <imageTopicName>image_raw</imageTopicName>
                <cameraInfoTopicName>camera_info</cameraInfoTopicName>
                <!-- 是 car_camera.xacro 中的link名字 -->
                <frameName>camera</frameName>
                <hackBaseline>0.07</hackBaseline>
                <distortionK1>0.0</distortionK1>
                <distortionK2>0.0</distortionK2>
                <distortionK3>0.0</distortionK3>
                <distortionT1>0.0</distortionT1>
                <distortionT2>0.0</distortionT2>
            </plugin>
        </sensor>
    </gazebo>
</robot>
```

-----

### **Step 3**：深度相机

这个同雷达一样需要注意的地方有三个：
1. \<gazebo reference="support"\> ：深度相机的link名；
2. \<cameraName\> ：深度相机发布到的话题名；
3. \<frameName\> ：深度相机的link名 ；

```xml
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 这里偷懒了，将雷达的支架设置成了深度相机 -->
    <gazebo reference="support">  
        <sensor type="depth" name="camera">
            <always_on>true</always_on>
            <update_rate>20.0</update_rate>
            <camera>
                <horizontal_fov>${60.0*PI/180.0}</horizontal_fov>
                <image>
                    <format>R8G8B8</format>
                    <width>640</width>
                    <height>480</height>
                </image>
                <clip>
                    <near>0.05</near>
                    <far>8.0</far>
                </clip>
            </camera>
            <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
                <!-- 相机数据输出的topic名 -->
                <cameraName>depth_camera</cameraName>
                <alwaysOn>true</alwaysOn>
                <updateRate>10</updateRate>
                <imageTopicName>rgb/image_raw</imageTopicName>  <!-- 深度相机也具备一般摄像头的功能，也可以采集一般图像 -->
                <depthImageTopicName>depth/image_raw</depthImageTopicName>
                <pointCloudTopicName>depth/points</pointCloudTopicName>
                <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
                <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
                <!-- 此处为连杆名 -->
                <!-- 此处的修改是为了后面点云坐标系调整 -->
                <frameName>support_depth</frameName>
                <baseline>0.1</baseline>
                <distortion_k1>0.0</distortion_k1>
                <distortion_k2>0.0</distortion_k2>
                <distortion_k3>0.0</distortion_k3>
                <distortion_t1>0.0</distortion_t1>
                <distortion_t2>0.0</distortion_t2>
                <pointCloudCutoff>0.4</pointCloudCutoff>
            </plugin>
        </sensor>
    </gazebo>
</robot>
```

-----

### **Step 4**：将空间嵌入到模型中
修改car.xacro文件
```xml
<robot name="car" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 添加惯性矩阵文件 -->
    <xacro:include filename="head.xacro" />

    <!-- 添加小车部件文件 -->
    <xacro:include filename="car_base.xacro" />
    <xacro:include filename="car_camera.xacro" />
    <xacro:include filename="car_laser.xacro" />
    
    <!-- 将运动控制的 move.xacro 文件集成进来 -->
    <xacro:include filename="gazebo/move.xacro" />

    <!-- 集成雷达 -->
    <xacro:include filename="gazebo/laser.xacro" />
    <!-- 集成摄像头 -->
    <xacro:include filename="gazebo/camera.xacro" />
    <!-- 集成深度相机 -->
    <xacro:include filename="gazebo/kinect.xacro" />

</robot>
```
-----

### **Step 6**：修改 rviz_sensor.launch 文件
**rviz_sensor.launch**
```xml
<launch>
    <node pkg="rviz" type="rviz" name="rviz" />

    <!-- 关节以及机器人状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

    <!-- 为了点云数据添加的坐标变化 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="potin_cloud_tf" args="0 0 0 -1.57 0 -1.57 /support /support_depth"/>

</launch>
```

-----

### **Step 5**：启动gazebo & rviz
```shell
$ roscore
$ source devel/setup.bash 
$ roslaunch demo car.launch
$ roalaunch demo rviz_sensor.launch
```

1. 雷达："Add" -> "LaserScan" -> "Topic" -> "/scan"；
2. 摄像头：“Add” -> “Camera” -> “Image Topic” -> “/camera/image_raw”；
3. 深度相机："Add" -> “Camera” -> "Image Topic" -> "/depth_camera/depth/image_raw" 或者 "/depth_camera/rgb/image_"；
4. 深度点云："Add" -> "DepthCloud" -> "Depth Map Topic" -> "/depth_camera/depth/image_raw"；
 
在雷达选项中的 “Size(m)” 值表示一个点的的长度是 0.05 m。

【注意】如果不对点云数据坐标系进行修正，那么点云数据是错位的，本应该显示的是机器人正前方，但刚打开的时候是机器人的头顶。这是因为在kinect中的（RGB、深度图像） 和 （点云） 使用了两套不同的坐标系，这里使用了 tf2_ros::static_transfor_publisher 的包直接转化两个坐标系关系。

对 cmd_vel 这个话题发布消息，让小车原地旋转来检查仿真情况：
```shell
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
        x: 0.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.3" 

```

----