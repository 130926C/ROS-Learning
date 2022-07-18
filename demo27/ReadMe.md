### demo27

机器人系统仿真主要包括三大部分：
1. 机器人建模 URDF；
2. 创建仿真环境 Gazebo；
3. 感知环境 Rviz；

如果安装的是desktop-full版本的话上述三个组建是自带的，否则可以使用如下命令安装：
```shell
$ sudo apt install ros-noetic-rivz
$ sudo apt install gazeboll
$ sudo apt install libgazeboll-dev
```

课程的部分资料链接：
```xml
https://github.com/zx595306686/sim_demo.git
```

在 Create Catkin Package 步骤中需要导入以下两个功能包：
```shell
urdf xacro
```

在生成的 demo 文件夹下新建三个文件夹：
1. launch：存放用来描述机器人的xml文件；
2. urdf：存放建模用的文件；
3. meshes：给机器人贴皮肤的；
4. config：rviz的配置目录；



在上面的urdf中新建两个文件夹：
1. urdf：存放urdf文件；
2. xacro：存放升级版urdf文件；

在 urdf-urdf 文件夹下创建一个机器人描述文件：
**test_robot.urdf**
```xml
<robot name="mycar">
	<link name="base_link">
		<visual>
			<geometry>
				<box size="0.5 0.2 0.1" />
			</geometry>
		</visual>
	</link>
</robot>
```

在 launch 文件夹下创建一个文件，可以不和urdf文件名对应上：
**test_robot.launch**
```xml
<launch>
	<!-- 在参数服务器上加载 urdf 文件 -->
	<param name="robot_description" textfile="$(find demo)/urdf/urdf/test_robot.urdf" />
	<!-- 启动rviz -->
	<node pkg="rviz" type="rviz" name="rviz" />
</launch>
```
【注】如果无法启动rivz，那么直接在cmd中启动也是等价的。

启动rviz后，在 “Add” 中选择 “RobotModel” 。

如果右侧工具栏中 “RobotModel”出现红色报错，可以将 “Global Option” 中的 “Fixed Frame” 坐标系手动改掉，应该改成 **test_robot.urdf** 文件中的 “link” 标签值，如 ”base_link“。

也可以使用 “Add” -> "Axes" 添加一个坐标系查看信息。

知乎链接：https://zhuanlan.zhihu.com/p/361126919

