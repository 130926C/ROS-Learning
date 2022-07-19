## demo 36
将xacro编写的小车模型添加惯性矩阵和碰撞检测，并在 gazebo 中现实。

需要实现以下几步：
1. 对xacro文件封装惯性矩阵；
2. 对每一个link添加collision和inertial标签，并重置颜色属性；
3. 在launch中启动gazebo并添加机器人；

-----

### **Sted 1** 封装惯性矩阵算法

这部分在视频教程中是直接复制连接的，知乎**赵虚左**的文章[ROS入门教程-理论与实践（6.6.3 URDF集成Gazebo实操）](https://zhuanlan.zhihu.com/p/362287335)中直接复制过来就可以。

**head.xacro**
```xml
<robot name="base" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- Macro for inertia matrix -->
    <xacro:macro name="sphere_inertial_matrix" params="m r">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                iyy="${2*m*r*r/5}" iyz="0" 
                izz="${2*m*r*r/5}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
        </inertial>
    </xacro:macro>

    <xacro:macro name="Box_inertial_matrix" params="m l w h">
       <inertial>
               <mass value="${m}" />
               <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                   iyy="${m*(w*w + l*l)/12}" iyz= "0"
                   izz="${m*(w*w + h*h)/12}" />
       </inertial>
   </xacro:macro>
</robot>
```

-----

### **Step 2** 对小车的底盘、摄像头、雷达部件写入属性

这里的小车模型基本模型从demo30中复制过来即可，并执行以下操作
1. 对每个link添加 collision 标签；
2. 在 link 中添加 xacro 宏展开；
3. 在 link 外添加 gazebo 颜色渲染标签；

**car_base.xacro**
```xml
<robot name="car_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 0.通用参数 -->
    <xacro:property name="PI" value="3.1415926" />
    
    <!-- 1.footprint -->
    <!--   footprint可以不设置惯性矩阵或者碰撞检测，因为是虚拟的点-->
    <xacro:property name="footprint_radius" value="0.0001" />
    <link name="footprint" >
        <visual>
            <geometry>
                <sphere radius="${footprint_radius}"/>
            </geometry>
        </visual>
    </link>

    <!-- 2.car base -->
    <xacro:property name="car_base_radius" value="0.1" />
    <xacro:property name="car_base_hight" value="0.08" />
    <xacro:property name="car_base_grand_dis" value="0.015" />
    <!-- 给小车添加一个质量 -->
    <xacro:property name="car_base_mass" value="2.0" />
    <xacro:property name="car_base_bias_z" value="${(car_base_hight / 2 + car_base_grand_dis)}" />
    <link name="car_base">
        <visual>
            <geometry>
                <cylinder radius="${car_base_radius}" length="${car_base_hight}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="car_base_color">
                <color rgba="0.5 0.5 0.5 0.5" />
            </material>
        </visual>

        <!-- 因为是规则的几何形状，所以碰撞参数设置和isual一样的即可 -->
        <collision>
            <geometry>
                <cylinder radius="${car_base_radius}" length="${car_base_hight}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </collision>

        <!-- 使用封装好的 head.xacro 文件来设置 inertial-->
        <xacro:cylinder_inertial_matrix m="${car_base_mass}" r="${car_base_radius}" h="${car_base_hight}" />
    </link>
    <!-- 设置颜色 -->
    <gazebo reference="car_base">
        <material>Gazebo/Yellow</material>
    </gazebo>   

    <joint name="car_base_joint_footprint" type="fixed">
        <parent link="footprint" />
        <child link="car_base" />
        <origin xyz="0 0 ${car_base_bias_z}" rpy="0 0 0" />
        <axis xyz="0 0 0" />
    </joint>


    <!-- 3.驱动轮-->
    <xacro:property name="drive_wheel_radius" value="0.0325" />
    <xacro:property name="drive_wheel_width" value="0.015" />
    <!-- 添加车轮质量 -->
    <xacro:property name="drive_whell_mass" value="0.05" />
    <xacro:property name="drive_wheel_bias_z" value="${(car_base_hight/2 + car_base_grand_dis - drive_wheel_radius) * -1}" />
    <xacro:macro name="drive_wheel_func" params="wheel_name flag">
        <link name="drive_${wheel_name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
                <material name="drive_wheel_color">
                    <color rgba="1 0 0 0.5" />
                </material>
            </visual>
            <!-- 添加碰撞检测 -->
            <collision>
                <geometry>
                    <cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
            </collision>
            <!-- 设置惯性矩阵 -->
            <xacro:cylinder_inertial_matrix m="${drive_whell_mass}" r="${drive_wheel_radius}" h="${drive_wheel_width}" />
        </link>

        <gazebo reference="drive_${wheel_name}_wheel">
            <material>Gazebo/Red</material>
        </gazebo>

        <joint name="${wheel_name}_joint_base" type="continuous">
            <parent link="car_base" />
            <child link="drive_${wheel_name}_wheel" />
            <origin xyz="0 ${0.1 * flag} ${drive_wheel_bias_z}" rpy="0 0 0"/>
            <axis xyz="0 1 0" />
        </joint>
    </xacro:macro>

    <xacro:drive_wheel_func wheel_name="left" flag="1" />
    <xacro:drive_wheel_func wheel_name="right" flag="-1" />

    <!-- 4.从动轮 -->
    <xacro:property name="small_wheel_radius" value="0.0075" />
    <xacro:property name="small_wheel_bias_z" value="${(car_base_hight / 2 + car_base_grand_dis - small_wheel_radius) * -1}" />
    <xacro:property name="small_wheel_mass" value="0.01" />
    <xacro:macro name="small_wheel_func" params="small_wheel_name flag" >
        <link name="small_${small_wheel_name}_wheel">
            <visual>
                <geometry>
                    <sphere radius="${small_wheel_radius}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <material name="small_wheel_color">
                    <color rgba="0 1 1 0.5"/>
                </material>
            </visual>

            <!-- 添加碰撞检测 -->
            <collision>
                <geometry>
                    <sphere radius="${small_wheel_radius}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0" />
            </collision>
            <xacro:sphere_inertial_matrix m="${small_wheel_mass}" r="${small_wheel_radius}" />
        </link>

        <gazebo reference="small_${small_wheel_name}_wheel">        
            <material>Gazebo/Blue</material>
        </gazebo>

        <joint name="small_${small_wheel_name}_wheel_joint_car_base" type="continuous">
            <parent link="car_base" />
            <child link="small_${small_wheel_name}_wheel" />
            <origin xyz="${0.08*flag} 0 ${small_wheel_bias_z}" rpy="0 0 0"/>
            <axis xyz="0 1 0" />
        </joint>
    </xacro:macro>

    <xacro:small_wheel_func small_wheel_name="front" flag="1" />
    <xacro:small_wheel_func small_wheel_name="back" flag="-1" />
</robot>
```

**car_camera.xacro**
```xml
<robot name="car_camera" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="camera_length" value="0.02" />
    <xacro:property name="camera_width" value="0.05" />
    <xacro:property name="camera_height" value="0.05" />
    <!-- 添加质量 -->
    <xacro:property name="camera_mass" value="0.02" />

    <xacro:property name="camera_x_bias" value="0.08" />
    <xacro:property name="camera_y_bias" value="0" />
    <xacro:property name="camera_z_bias" value="${car_base_hight / 2 + camera_height / 2}" />

    <link name="camera">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="camera_color">
                <color rgba="1 1 1 0.5" />
            </material>
        </visual>

        <!-- 添加碰撞检测 -->
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </collision>
        <!-- 惯性矩阵 -->
        <xacro:Box_inertial_matrix m="${camera_mass}" l="${camera_length}" w="${camera_width}" h="${camera_height}" />
    </link>

    <!-- 添加颜色 -->
    <gazebo reference="camera" >
        <material>Gazebo/Gray</material>
    </gazebo>   

    <joint name="camera_joint_car_base" type="fixed">
        <parent link="car_base" />
        <child link="camera" />
        <origin xyz="${camera_x_bias} ${camera_y_bias} ${camera_z_bias}" rpy="0 0 0" />
    </joint>
</robot>
```

**car_lader.xacro**
```xml
<robot name="car_lader" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 1.雷达支架 -->
    <xacro:property name="support_radius" value="0.01" />
    <xacro:property name="support_length" value="0.15" />
    <xacro:property name="support_mass" value="0.1" />
    <!-- 雷达支架偏移量 --> 
    <xacro:property name="support_x" value="0" />
    <xacro:property name="support_y" value="0" />
    <xacro:property name="support_z" value="${car_base_hight/2 + support_length / 2}" />


    <!-- 2.雷达尺寸 -->
    <xacro:property name="laser_radius" value="0.03" />
    <xacro:property name="laser_length" value="0.05" />
    <xacro:property name="laser_mass" value="0.05" />
    <!-- 雷达偏移量 -->
    <xacro:property name="laser_x" value="0" />
    <xacro:property name="laser_y" value="0" />
    <xacro:property name="laser_z" value="${support_length / 2 + laser_length / 2}" />
    <!-- link 和 joint 实现 -->
    <!-- 连接杆 -->
    <link name="support">
        <visual>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
            <material name="yellow">
                <color rgba="0.8 0.5 0 0.5" />
            </material> 
        </visual>

        <!-- 添加碰撞检测 -->
        <collision>
            <geometry>
                <cylinder radius="${support_radius}" length="${support_length}" />
            </geometry>
        </collision>
        <!-- 添加碰撞矩阵 -->
        <xacro:cylinder_inertial_matrix m="${support_mass}" r="${support_radius}" h="${support_length}"/>
    </link>

    <!-- 添加颜色 -->
    <gazebo reference="support" >
        <material>Gazebo/White</material>
    </gazebo>

    <joint name="support2base" type="fixed">
        <parent link="car_base" />
        <child link="support" />
        <origin xyz="${support_x} ${support_y} ${support_z}" rpy="0 0 0" />
    </joint>

    <!-- 雷达 -->
    <link name="laser">
        <visual>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 0.5" />
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${laser_radius}" length="${laser_length}" />
            </geometry>
        </collision>
        <xacro:cylinder_inertial_matrix m="${laser_mass}" r="${laser_radius}" h="${laser_length}" />
    </link>

    <gazebo reference="laser">
        <material>Gazebo/Black</material>
    </gazebo>

    <joint name="lader2support" type="fixed">
        <parent link="support" />
        <child link="laser" />
        <origin xyz="${laser_x} ${laser_y} ${laser_z}" rpy="0 0 0" />
    </joint>
</robot>
```

-----

### **Step 3** 添加一个环境模型

这个模型在很早之前的课程中提到过，在这个demo中的world文件夹下，或和可以在 [这里](https://github.com/zx595306686/sim_demo) 下载。

此时，你的文件结构应该如下所示：
```shell
$ tree
    .
    ├── CMakeLists.txt
    ├── include
    │   └── demo
    ├── launch
    │   └── car.launch          # 启动文件（到这一步截至还是空的）
    ├── package.xml
    ├── src
    ├── urdf
    │   ├── car_base.xacro      # 小车模型
    │   ├── car_camera.xacro
    │   ├── car_lader.xacro
    │   ├── car.xacro
    │   └── head.xacro          # 惯性矩阵宏定义
    └── world
        └── box_house.world     # 仿真环境文件
```

-----

### **Step 4** 编写启动文件

**car.launch**
```xml
<launch>
    <!-- 在服务器中载入xacro文件 -->    
    <param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/car.xacro" />

    <!-- 启动gazebo仿真环境 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <!-- 【注意】这个 world_name 是固定写法，不可以更改-->
        <arg name="world_name" value="$(find demo)/world/box_house.world"/>
    </include>

    <!-- 添加机器人模型  -->
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_node" args="-urdf -model car -param robot_description" />

</launch>
```

【注意】如果此时只想让小车载入到一个空的环境中，那么应该如下写：
```xml
<launch>
    <!-- 在服务器中载入xacro文件 -->    
    <param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/car.xacro" />

    <!-- 启动gazebo仿真环境 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 添加机器人模型  -->
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_node" args="-urdf -model car -param robot_description" />

</launch>
```

-----

### **Step 5** 终端启动 

```shell
$ source devel/setup.bash
$ roslaunch demo car.launch 
```