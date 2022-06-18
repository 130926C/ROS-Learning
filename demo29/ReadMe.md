### demo29

URDF 中存在以下两个主要问题：
1. 坐标位置换算太复杂，并且参数被写死，不容易改变参数；
2. 大量的代码其实可以复用；

为了解决上面两个问题，ROS提供了 xacro 语言。

使用命令行运行文件：
```shell
$ rosrun xacro xacro test.xacro
```

将运行结果写入指定文件中：
```shell
$ rosrun xacro xacro test.xacro > test.urdf
```

【注】在xacro中的定义是没有输出的，只有调用的时候才有结果。

--------

### xacro头部信息
```xml
<robot name="mycar" xmlns:xacro="http://www.ros.org/wiki/xacro">

</robot>
```

-------

### 属性定义和调用
```xml
<!-- 1.属性定义 -->
<xacro:property name="PI" value="3.1415926" />
<xacro:property name="radius" value="0.03" />
<!-- 属性调用 -->
<myUseProperty name="${PI}"/>
<!-- 算术运算 -->
<myCalculater name="${PI * 3}" />
```

-------

**宏定义和调用**
```xml
<!-- 2.宏定义 -->
<xacro:macro name="getSum" params="num1 num2" >
<result value="${num1 + num2}" />
</xacro:macro>
<!-- 宏调用 -->
<xacro:getSum num1="1" num2="5" />
```

--------

**文件包含**
```xml
<!-- 3.文件包含 -->
<xacro:inlcude filename="others.xacro" />
```

-------

### 将xacro文件集成到 rviz 上
有两种方法可以将xacro集成到rviz上：
方法一：先生成urdf文件，然后用urdf的launch集成；
```xml
<launch>
	<param name="robot_description" textfile="$(find demo)/urdf/xacro/car_base.urdf" />
	<node pkg="rviz" type="rviz" name="rviz" />
	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
	<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
</launch>
```

方法二【推荐】：直接用xacro的launch文件集成；
```xml
<launch>
	<param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/xacro/car_base.xacro"/>
	<node pkg="rviz" type="rviz" name="rviz" />
	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
	<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
</launch>
```

二者之间最大的区别在于 param 标签中使用的是 command 属性。

-------

### 使用xacro创建一个小车

```xml
<robot name="mycar" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<!-- 0.封装全局变量 -->
	<xacro:property name="PI" value="3.1415926" />
	
	<!-- 1.footprint -->
	<!-- 需要封装一个小球直径-->
	<xacro:property name="footprint_r" value="0.001"/>
	
	<link name="base_footprint">
		<visual>
			<geometry>
				<sphere radius="${footprint_r}"/>
			</geometry>
		</visual>
	</link>
	
	<!-- 2.底盘 -->
	<xacro:property name="car_base_radius" value="0.1" />
	<xacro:property name="car_base_length" value="0.08" />
	<xacro:property name="grand_dis" value="0.015" />
	<xacro:property name="car_base_bias_z" value="${car_base_length / 2 + grand_dis}" />
	
	<link name="car_base">
		<visual>
			<geometry>
				<cylinder radius="${car_base_radius}" length="${car_base_length}"/>
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0" />
			<material name="gray">
				<color rgba="0.5 0.5 0.5 0.5" />
			</material>
		</visual>
	</link>
	
	<joint name="foot_print2car_base" type="fixed">
		<parent link="base_footprint" />
		<child link="car_base" />
		<origin xyz="0 0 0.055" rpy="0 0 0" />
	</joint>
	
	<!-- 3.车轮 -->
	<xacro:property name="wheel_radius" value="0.0325" />
	<xacro:property name="wheel_length" value="0.015" />
	<xacro:property name="wheel_bias_z" value="${-1 * (car_base_length / 2 + grand_dis - wheel_radius)}" />
	
	<!--
	wheel_name: left or right
	flag: 1 or -1
	-->
	<xacro:macro name="wheel_func" params="wheel_name flag">
	
		<link name="${wheel_name}_wheel">
			<visual>
				<geometry>
					<cylinder radius="${wheel_radius}" length="${wheel_length}" />
				</geometry>
				<origin xyz="0 0 0" rpy="${PI / 2} 0 0" />
				<material name="red">
					<color rgba="1 0 0 0.5" />
				</material>
			</visual>
		</link>
		
		<joint name="${wheel_name}2link" type="continuous">
			<parent link="car_base" />
			<child link="${wheel_name}_wheel" />
			<origin xyz="0 ${0.1 * flag} ${wheel_bias_z}" rpy="0 0 0" />
			<axis xyz="0 1 0" />
		</joint>
	</xacro:macro>
	
	<xacro:wheel_func wheel_name="left" flag="1" />
	<xacro:wheel_func wheel_name="right" flag="-1" />
	
	<!-- 4.万向轮 -->
	<xacro:property name="small_wheel_radius" value="0.0075"/>
	<xacro:property name="small_wheel_bias_z" value="${(car_base_length / 2 + grand_dis - small_wheel_radius) * -1 }" />
	
	<xacro:macro name="small_wheel_func" params="small_wheel_name flag">
		<link name="${small_wheel_name}_wheel">
			<visual>
				<geometry>
					<sphere radius="${small_wheel_radius}" />
				</geometry>
				<origin xyz="0 0 0" rpy="0 0 0"/>
				<material name="blue">
					<color rgba="0 1 0 0.5" />
				</material>
			</visual>
		</link>
		
		<joint name="${small_wheel_name}_link" type="continuous">
			<parent link="car_base" />
			<child link="${small_wheel_name}_wheel" />
			<origin xyz="${0.08 * flag} 0 ${small_wheel_bias_z}" rpy="0 0 0" />
			<axis xyz="0 1 0" />
		</joint>
	</xacro:macro>
	
	<xacro:small_wheel_func small_wheel_name="front" flag="1" />
	<xacro:small_wheel_func small_wheel_name="back" flag="-1" />
</robot>
```


