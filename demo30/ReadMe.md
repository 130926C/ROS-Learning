### demo30

使用xacro的include方式组装一个小车。

### Step 0：launch文件
```xml
<launch>
	<param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/xacro/car.xacro"/>
	<node pkg="rviz" type="rviz" name="rviz" />
	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
	<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui"/>
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
</launch>
```

### Step 1：小车框架
```xml
<robot name="car" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<xacro:include filename="car_base.xacro" />
	<xacro:include filename="car_camera.xacro" />
	<xacro:include filename="car_lader.xacro" />
</robot>
```

### Step2：相机
```xml
<robot name="car_camera" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<xacro:property name="camera_length" value="0.02" />
	<xacro:property name="camera_width" value="0.05" />
	<xacro:property name="camera_height" value="0.05" />

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
	</link>

	<joint name="camera_joint_car_base" type="fixed">
		<parent link="car_base" />
		<child link="camera" />
		<origin xyz="${camera_x_bias} ${camera_y_bias} ${camera_z_bias}" rpy="0 0 0" />
	</joint>

</robot>
```

### Step3：雷达
```xml
<robot name="car_lader" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<!-- 1.雷达支架 -->
	<xacro:property name="support_radius" value="0.01" />
	<xacro:property name="support_length" value="0.15" />
	<!-- 雷达支架偏移量 -->
	<xacro:property name="support_x" value="0" />
	<xacro:property name="support_y" value="0" />
	<xacro:property name="support_z" value="${car_base_hight/2 + support_length / 2}" />

	<!-- 2.雷达尺寸 -->
	<xacro:property name="laser_radius" value="0.03" />
	<xacro:property name="laser_length" value="0.05" />
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
	</link>
	
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
	</link>

	<joint name="lader2support" type="fixed">
	<parent link="support" />
	<child link="laser" />
	<origin xyz="${laser_x} ${laser_y} ${laser_z}" rpy="0 0 0" />
	</joint>

</robot>
```

