## demo 51 
这个demo集成了 scout 小车、激光雷达、kinect V2 的模型使用。

主要在于理解 ros 中 link 和 frame 之间的关系。由于整个demo文件过大，特别是第三方package很大，所以下面提供了git链接，在运行之前需要在src目录下clone这些仓库。

```shell
$ git clone https://github.com/code-iai/iai_kinect2.git  # kinect
$ git clone https://github.com/wangxian4423/kinect_v2_udrf.git # kinect xacro 仓库
$ git clone https://github.com/ldrobotSensorTeam/ldlidar_sl_ros.git # 乐动机器激光雷达
$ git clone https://github.com/agilexrobotics/ugv_sdk.git  # scout 机器人底盘驱动
$ git clone https://github.com/agilexrobotics/scout_ros.git  # scout 机器人底盘 ROS package
```

下面是上述文件中改动的部分

-----

### 给小车添加 xacro 模型

src/scout_ros/scout_description/urdf/scout_v2.xacro

```xml
<?xml version="1.0"?>

<robot name="scout_v2" 
    xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:arg name="robot_namespace" default="/" /> 

    <xacro:include filename="$(find scout_description)/urdf/scout_wheel_type1.xacro" />
    <xacro:include filename="$(find scout_description)/urdf/scout_wheel_type2.xacro" />

    <!-- Variables -->
    <xacro:property name="M_PI" value="3.14159"/>

    <!-- Vehicle Geometries -->
    <xacro:property name="base_x_size" value="0.9250000" />
    <xacro:property name="base_y_size" value="0.380000" />
    <xacro:property name="base_z_size" value="0.210000" />

    <xacro:property name="wheelbase" value="0.498" />
    <xacro:property name="track" value="0.58306" />
    <xacro:property name="wheel_vertical_offset" value="-0.0702" />

    <xacro:property name="wheel_length" value="1.1653e-01" />
    <xacro:property name="wheel_radius" value="1.6459e-01" />

    <!-- Base link -->
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <mesh filename="package://scout_description/meshes/base_link.dae" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0.008" rpy="0 0 0" />
            <geometry>
                <box size="${base_x_size} ${base_y_size} ${base_z_size}"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 ${base_z_size/6}" rpy="0 0 0" />
            <geometry>
                <box size="${base_x_size/6} ${base_y_size*1.65} ${base_z_size/3}"/>
            </geometry>
        </collision>
    </link>

    <link name="base_footprint"/>

    <joint name="base_footprint_joint" type="fixed">
        <origin xyz="0 0 ${wheel_vertical_offset - wheel_radius}" rpy="0 0 0" />
        <parent link="base_link" />
        <child link="base_footprint" />
    </joint>

    <link name="inertial_link">
        <inertial>
            <mass value="40" />
            <origin xyz="0.0 0.0 0.0" />
            <inertia ixx="2.288641" ixy="0" ixz="0" iyy="5.103976" iyz="0" izz="3.431465" />
        </inertial>
    </link>

    <joint name="inertial_joint" type="fixed">
        <origin xyz="0 0 0" rpy="0 0 0" />
        <parent link="base_link" />
        <child link="inertial_link" />
    </joint>

    <!-- Scout wheel macros -->
    <!-- wheel labeled from 0 to 3, conter-clockwise, starting from front right wheel -->
    <!-- motor 1 and 2 (left side) are mechanically installed in a reversed direction -->
    <xacro:scout_wheel_type1 wheel_prefix="front_right">
        <origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="3.14 0 0" />
    </xacro:scout_wheel_type1>
    <xacro:scout_wheel_type2 wheel_prefix="front_left">
        <origin xyz="${wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
    </xacro:scout_wheel_type2>
    <xacro:scout_wheel_type1 wheel_prefix="rear_left">
        <origin xyz="${-wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
    </xacro:scout_wheel_type1>
    <xacro:scout_wheel_type2 wheel_prefix="rear_right">
        <origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="3.14 0 0" />
    </xacro:scout_wheel_type2>

    <!-- self add laser -->
    <link name="laser_link" >
        <visual>
            <geometry>
                <box size="0.1 0.1 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>
    <joint name="baselink2laser" type="fixed">
        <parent link="base_link" />
        <child link="laser_link" />
        <origin xyz="0.25 0 ${base_z_size / 2 }" rpy="0 0 3.14" />
    </joint>

    <!-- self add kinect v2-->
    <xacro:include filename="$(find kinect_v2)/urdf/kinect_v2.urdf.xacro" />
    <xacro:kinect_v2 parent="base_link" >
        <origin xyz="-0.35 0 ${base_z_size / 2 }" rpy="0 0 3.14" />
    </xacro:kinect_v2>

</robot>

```

-----

### 绑定激光雷达的link

src/ldlidar_sl_ros/launch/ld14.launch

```xml
<launch>
    <!-- LDROBOT LiDAR message publisher node -->
    <node name="ldlidar_publisher_ld14" pkg="ldlidar_sl_ros" type="ldlidar_sl_ros_node" output="screen">
        <param name="product_name" value="LDLiDAR_LD14"/>
        <param name="topic_name" value="scan"/>
        <param name="port_name" value ="/dev/ttyUSB0"/>
        <param name="frame_id" value="laser_link"/>
        <!-- Set laser scan directon: -->
        <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
        <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
        <param name="laser_scan_dir" type="bool" value="true"/>
        <!-- Angle crop setting, Mask data within the set angle range -->
        <!--    1. Enable angle crop fuction: -->
        <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
        <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
        <param name="enable_angle_crop_func" type="bool" value="false"/>
        <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
        <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
        <param name="angle_crop_min" type="double" value="135.0"/>
        <param name="angle_crop_max" type="double" value="225.0"/>
    </node>
    <!-- LDROBOT LiDAR message subscriber node -->
    <!-- node name="ldlidar_listener_ld14" pkg="ldlidar_sl_ros" type="ldlidar_sl_ros_listen_node" output="screen">
    <param name="topic_name" value="scan"/>
    </node -->
    <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
    <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
</launch>

```

----

