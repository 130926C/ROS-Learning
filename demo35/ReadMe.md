## demo 35
URDF与Gazebo集成
1. 创建功能包 & 导入相关依赖；
2. 编写urdf/xacro文件；
3. 启动Gazebo & 导入模型；

-----

**Step 1**
在创建的时候导入以下依赖：
```txt
roscpp urdf xacro gazebo_ros gazebo_ros_control gazebo_plugins
```

----

**Step 2**
编写一个矩形的机器人模型 box_robot.urdf：
```xml
<robot name="box_robot">
    <link name="base_link" >

        <!-- 可视化部分 -->
        <visual>
            <geometry>
                <box size="0.5 0.3 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <material name="robot_color">
                <color rgba="0.5 0.3 0 0.5" />
            </material>
        </visual>

        <!-- Gazebo 特有部分 -->
        <!-- 设置碰撞参数 -->
        <collision>
            <!-- 如果是标准几何体，直接复制上面的就可以 -->
            <geometry>
                <box size="0.5 0.3 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>

        <!-- 设置惯性矩阵 -->
        <inertial>
            <origin xyz="0 0 0" />  <!-- 重心偏移量 -->
            <mass value="2" />      <!-- 质量 -->
            <!-- 不同纬度上的惯性参数 -->
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
        </inertial>
    </link>

    <!-- Gazebo有自己的颜色设置 -->
    <gazebo reference="base_link">
        <material>Gazebo/Red</material>
    </gazebo>
</robot>
```
【注意】后面的gazebo颜色设置是放在link标签外面的。

---
**Step 3** 
编写启动Gazebo的launch文件：start.launch
```xml
<launch>
    <!-- 在参数服务器中载入urdf文件 -->
    <param name="robot_description" textfile="$(find demo)/urdf/box_robot.urdf"/>

    <!-- 启动gazebo仿真环境 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 在gazebo中添加机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_model_node" args="-urdf -model box -param robot_description" />

</launch>
```
这里的 spawn_model 参数比较多，如果想不起来怎么用可以下面的命令查看：
```shell
$ rosrun gazebo_ros spawn_model -h
```

----
**Step 4**
启动gazebo：
```shell
$ roslaunch demo start.launch
```

---

此外，还有几个标准物体的惯性矩阵封装成了xacro宏可以使用：

**球体**
```xml
<xacro:macro name="sphere_inertial_matrix" params="m r">
    <inertial>
        <mass value="${m}" />
        <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                 iyy="${2*m*r*r/5}" iyz="0" 
                 izz="${2*m*r*r/5}" />
    </inertial>
</xacro:macro>
```

**圆柱体**
```xml
<xacro:macro name="cylinder_inertial_matrix" params="m r h">
    <inertial>
        <mass value="${m}" />
        <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                 iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                 izz="${m*r*r/2}" /> 
    </inertial>
</xacro:macro>
```

**立方体**
```xml
<xacro:macro name="Box_inertial_matrix" params="m l w h">
    <inertial>
        <mass value="${m}" />
            <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                    iyy="${m*(w*w + l*l)/12}" iyz= "0"
                    izz="${m*(w*w + h*h)/12}" />
    </inertial>
</xacro:macro>
```
