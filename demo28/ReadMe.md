### demo28

URDF 是一个标准的 XML 文件，在 ROS 中预定义了一系列用来描述机器人的模型，主要分为两部分：
1. 连杆 link 标签，描述的是部件的形态；
2. 关节 joint 标签，描述的是部件之间的连接关系；

-----

### link 标签
用来描述机器人某个部件外观和属性（通常是刚体部分），如底盘、轮子、雷达、摄像头等。这个标签可以设置形状、尺寸、颜色、惯性矩阵、碰撞参数等。

1. 属性：只有一个 name 命名属性；
2. 子级标签：
* visual 外观描述
	*  geometry 形状
		* box、cylinder、radius、mesh
	* origin 偏移量和倾斜弧度
		* xyz、rpy
	* metrial 材料和颜色
		* name、color
* collision 碰撞属性
* Inertial 惯性矩阵

------

### 案例一：添加一个小车机器人
```xml
<robot name="mycar">
	<!-- 添加底盘 -->
	<link name="base_link">
		<!-- 给底盘增加可视化信息 -->
		<visual>
		
			<!-- 形状 -->
			<geometry>
				<!-- 可以选择下面四个中任意一个添加 -->
				<box size="0.3 0.2 0.1" />
				<cylinder radius="0.1" length="2" />
				<sphere radius="1" />
				<!-- 这个mesh文件是课程提供的资源 -->
				<mesh filename="package://demo/meshes/autolabor_mini.stl"/>
			</geometry>
			
			<!-- 位置偏移量和弧度 -->
			<origin xyz="0 0 0" rpy="1.57 0 1.57"/>
			<!-- 颜色 -->

			<material name="car_color">
				<color rgba="0.5 0.5 0.5 1" />
			</material>
		</visual>
	</link>
</robot>
```

其中，mesh标签的 filename 属性格式必须以 "package://" 开头。

launch 文件
```xml
<launch>
	<param name="robot_description" textfile="$(find demo)/urdf/urdf/robot.urdf" />
	
	<node pkg="rviz" type="rviz" name="rviz" args="$(find demo)/config/robot_config.rviz"/>
</launch>
```

-------

### joint 标签

用来描述机器人关节的运动学和动力学属性，还可以指定关节运动极限，机器人的两个部件是以关节的形式链接的（从属关系分别被称为parent link 和 child link）。

1. 属性：
	* name：关节的名字
	* type：关节的运动形式
		* continuous：旋转关节，可以绕单轴无限旋转；
		* revolute：旋转关节，有角度限制的旋转；
		* prismatic：滑动关节，沿某一轴线移动，有位移限制；
		* planer：平面关节，允许在平面正交方向上旋转或平移；
		* floating：浮动关节，允许进行平移、旋转；
		* fixed：固定关节，不允许任何运动的特殊关节；
2. 子级标签：
	* parent：（必须的）
		* link：父级link的名字
	* child：（必须的）
		* link：子级link的名字
	* origin：
		* xyz、rpy
	* axis：
		* xyz设置围绕哪个关节轴运动

-----

### 案例二：自定义组合机器人
```xml
<!-- 添加一个自定义形状的机器人，主要有底盘和摄像头两部分组成 -->
<robot name="mycar">
	<!-- 1.底盘link -->
	<link name="base_link">
		<visual>
			<geometry>
				<box size="0.3 0.2 0.1"/>
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0"/>
			<material name="gray">
				<color rgba="0.5 0.5 0.5 0.5"/>
			</material>
		</visual>
	</link>
	
	<!-- 2.摄像头link -->
	<link name="camera">
		<visual>
			<geometry>
				<box size="0.02 0.05 0.05" />
			</geometry>
			<origin xyz="0 0 0.025" rpy="0 0 0"/>
			<material name="red">
				<color rgba="1 0 0 0.5"/>
			</material>
		</visual>
	</link>
	
	<!-- 3.链接两个link的关节joint信息 -->
	<joint name="camera2base" type="continuous">
		<!-- 父 link -->
		<parent link="base_link" />
		<!-- 子 link -->
		<child link="camera" />
		<!-- 设置偏移量 -->
		<origin xyz="0.12 0 0.05" rpy="0 0 0" />
		<!-- 关节旋转的坐标轴 -->
		<axis xyz="0 0 1" />
	</joint>

</robot>
```

launch 文件
```xml
<launch>
	<!-- 1.在参数服务器中载入 urdf 文件 -->
	<param name="robot_description" textfile="$(find demo)/urdf/urdf/robot_joint.urdf"/>
	<!-- 2.启动 rivz -->
	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo)/config/robot_config.rviz" />
	
	<!-- ROS 已经提供了发布不同部件和节点之间相对位置关系的功能包 -->
	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher"/>
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>
	
	<!-- 添加控制关节运动的节点 -->
	<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
	
	</launch>
```

----------

### 案例三：优化 URDF 文件
由于在ROS中新建的第一个base是按照质心“嵌入”到地里的，这可能会导致实际应用过程中这种情况并不符合实际。  

一个可行的解决方案是：添加一个尺寸极小的link作为整个系统的base，再去关联原先系统的link。通常情况下会把这个link叫做 “base_footprint”。

URDF文件
```xml
<robot name="mycar">
	<!-- base_footpritn -->
	<link name="base_footprint">
		<visual>
			<geometry>
				<box size="0.001 0.001 0.001" />
			</geometry>
		</visual>
	</link>

	<!-- 1.底盘link -->
	<link name="car_base">
		<visual>
			<geometry>
				<box size="0.3 0.2 0.1"/>
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0"/>
			<material name="blue">
				<color rgba="0.5 0.5 0.5 0.5"/>
			</material>
		</visual>
	</link>

	<!-- 2.摄像头link -->
	<link name="camera">
		<visual>
			<geometry>
				<box size="0.02 0.05 0.05" />
			</geometry>
			<origin xyz="0 0 0.025" rpy="0 0 0"/>
			<material name="yellow">
				<color rgba="1 0 0 0.5"/>
			</material>
		</visual>
	</link>

	<!-- 3.链接两个link的关节joint信息 -->
	<joint name="camera2base" type="continuous">
		<parent link="car_base" />
		<child link="camera" />
		<origin xyz="0.12 0 0.05" rpy="0 0 0" />
		<axis xyz="0 0 1" />
	</joint>

	<!-- 添加base_joint -->
	<joint name="link2footprint" type="fixed" >
		<parent link="base_footprint" />
		<child link="car_base" />
		<origin xyz="0 0 0.05" rpy="0 0 0"/>
		<axis xyz="0 0 0" />
	</joint>
	
</robot>
```

launch 文件
```xml
<launch>
	<param name="robot_description" textfile="$(find demo)/urdf/urdf/robot_base_footlink.urdf"/>
	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo)/config/robot_config.rviz" />

	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher"/>
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>
</launch>
```

-------

### 案例四：URDF文件练习
创建一个四轮圆柱状的机器人模型，由两个驱动轮和两个万向轮组成。

推荐标准：如果要设置偏移量，最好使用joint来设置，避免直接使用link设置，这样可以减少后期坐标转化的工作量。让 "origin" 标签中的 xyz全都为0。
```xml
<link name="xxx">
	<visual>
		<origin xyz="0 0 0" rpy="1.5708 0 0" />
	</visual>
</link>
```


urdf 文件
```xml
<robot name="mycar">
	<!-- base_foorprint -->
	<link name="base_footprint" >
		<visual>
			<geometry>
				<box size="0.001 0.001 0.001" />
			</geometry>
		</visual>
	</link>

	<!-- 添加底座 -->
	<link name="car_base">
		<visual>
			<geometry>
				<cylinder radius="0.1" length="0.08" />
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0"/>
			<material name="gray">
				<color rgba="0.5 0.5 0.5 0.5" />
			</material>
		</visual>
	</link>

	<!-- 添加驱动轮 -->
	<!-- 左驱动轮 -->
	<link name="left_wheel">
		<visual>
			<geometry>
				<cylinder radius="0.0325" length="0.015" />
			</geometry>
			<origin xyz="0 0 0" rpy="1.5708 0 0" />
			<material name="black">
				<color rgba="1 1 1 0.5"/>
			</material>
		</visual>
	</link>

	<!-- 右驱动轮 -->
	<link name="right_wheel">
		<visual>
			<geometry>
				<cylinder radius="0.0325" length="0.015" />
			</geometry>
			<origin xyz="0 0 0" rpy="1.5708 0 0" />
			<material name="black">
				<color rgba="1 1 1 0.5" />
			</material>
		</visual>
	</link>

	<!-- 添加万象轮 -->
	<link name="front_wheel">
		<visual>
			<geometry>
				<sphere radius="0.0075" />
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0"/>
			<material name="black">
				<color rgba="1 1 1 0.5"/>
			</material>
		</visual>
	</link>
	
	<link name="back_wheel">
		<visual>
			<geometry>
				<sphere radius="0.0075" />
			</geometry>
			<origin xyz="0 0 0" rpy="0 0 0" />
			<material name="black">
				<color rgba="1 1 1 0.5" />
			</material>
		</visual>
	</link>

	<!-- joint：base_footprint & base_line -->
	<joint name="base_footprint2car_base" type="fixed">
		<parent link="base_footprint"/>
		<child link="car_base"/>
		<origin xyz="0 0 0.055" rpy="0 0 0" />
	</joint>

	<!-- joint：左驱动轮 -->
	<joint name="left2base" type="continuous">
		<parent link="car_base" />
		<child link="left_wheel" />
		<origin xyz="0 0.1 -0.0225" rpy="0 0 0" />
		<axis xyz="0 1 0"/>
	</joint>

	<!-- joint：右驱动轮 -->
	<joint name="right2base" type="continuous">
		<parent link="car_base" />
		<child link="right_wheel" />
		<origin xyz="0 -0.1 -0.0225" rpy="0 0 0" />
		<axis xyz="0 1 0"/>
	</joint>

	<!-- 前万向轮 -->
	<joint name="front2base" type="continuous" >
		<parent link="car_base" />
		<child link="front_wheel" />
		<origin xyz="0.08 0 -0.0475" rpy="0 0 0" />
		<axis xyz="0 1 0" />
	</joint>

	<!-- 后万向轮 -->
	<joint name="back2base" type="continuous" >
		<parent link="car_base" />
		<child link="back_wheel" />
		<origin xyz="-0.08 0 -0.0475" rpy="0 0 0" />
		<axis xyz="0 1 0" />
	</joint>

</robot>
```

launch  文件
```xml
<launch>
	<param name="robot_description" textfile="$(find demo)/urdf/urdf/robot_test.urdf"/>

	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo)/config/robot_config.rviz" />

	<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />

	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

	<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
</launch>
```

核心内容是z轴的转换部分。

--------

### URDF 工具包使用
在使用之前需要安装功能包：
```shell
$ sudo apt install liburdfdom-tools
```

**chech_urdf**
检查urdf文件是否存在语法错误。如果没有问题会输出下面的文本
```shell
$ $ check_urdf src/demo/urdf/urdf/robot_test.urdf 
	robot name is: mycar
	---------- Successfully Parsed XML ---------------
	root Link: base_footprint has 1 child(ren)
	    child(1):  car_base
	        child(1):  back_wheel
	        child(2):  front_wheel
	        child(3):  left_wheel
	        child(4):  right_wheel
```

**urdf_to_graphiz**
将urfd转化为pdf文件
```shell
$  urdf_to_graphiz src/demo/urdf/urdf/robot_test.urdf 
Created file mycar.gv
Created file mycar.pdf
```

会在当前工作目录下生成 "mycar.pdf" 文件。

-------

