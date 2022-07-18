### demo15

launch 文件。

在这一以小乌龟案例为例，演示如何使用launch文件来启动这个例子。launch文件是后面很重要的内容，语法也很简单，务必熟练。

-----

Step 1：在Create catkin make步骤中添加 turtlesim 依赖
```cpp
rospy roscpp std_msgs turtlesim
```
---------

Step 2：编写launch文件
```launch
<launch>
	<!--需要启动的节点-->
	<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>

</launch>
```

在使用roslaunch的时候会自动检查是否已经启动了roscore，如果启动了就不会再启动，没启动的话自动启动。


在ros中的launch标签只有一个可选属性 “deprecated” 用来声明这个launch文件已经过时了，不建议使用，如果执行了这个launch也是可以的，但会在cmd中输出一份自己写的警告。
```launch
<launch deprecated="此文件已经过时了，不建议使用">
	<!--需要启动的节点-->
	<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>

</launch>
```

```shell
WARNING: [/home/lucks/Desktop/code/ROS/demo15/src/demo/launch/start_turtle.launch] DEPRECATED: 此文件已经过时了，不建议使用
ROS_MASTER_URI=http://localhost:11311

process[my_turtle-1]: started with pid [23334]
process[my_key-2]: started with pid [23335]
```
尽管有警告输出，但仍然可以执行（前提是代码没有问题）。

【注】launch文件启动时无法保证node按照顺序启动，因为ros是多进程的框架。

--------

ROS 标签和值

```launch
	<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" respawn="true" />
```

常用标签
pkg：包名，可以是自定义的demo
type：节点类型（与节点相同的可执行文件名，就是add_executable中起的名字）
name：节点名称，这个名称值的是给这个节点起的别名，并不需要和代码中的节点名一致
output：日值输出重定向，“log”或者”screen“

其他标签
args：“XXX XXX XXX”，将参数传递给节点
```launch
<node pkg="demo" type="launch_node" name="my_launch" output="screen" args="_length:=5" />
```

```shell
[ INFO] [1650365366.330382010]: Param:/my_launch/length, value[1]=5.00
```

machine：机器名，可以在分布式设备上启动节点。

respawn="true | false"：节点意外退出后是否自动重启。但可以通过ctrl+C的方式强制退出。

respawn_delay="N"：节点关闭后延迟N秒重启。

required="true | false"：节点是否是必须的，如果是true则该节点被关闭后退出整个launch。
		【这个的权限要高于 respawn，当两者都为true的时候，退出了也无法重启。】

ns="xxx"：指定命名空间，用于避免重名问题

```launch
	<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" ns="test"/>
```

```shell
$ rosnode list
/test/my_turtle
/my_key
/rosout
```

-------

launch文件复用

在一个新的launch文件中需要调用其他launch文件中的内容，即不使用复制粘贴的形式调用其他节点。
1. 可以降低代码量。
2. 当对节点的启动配置改变时可以传递到其他launch文件中。

```launch
<!-- 需要复用 start_turtle.launch 中的节点以及配置-->

<launch>
	<include file="$(find demo)/launch/start_turtle.launch" />
	<!-- 启动其他节点 -->
</launch>
```
其中：
```launch
file="$(find 功能包名)/launch/需要包含的文件名"
```

-------

remap 修改话题名称

可以将节点的话题名修改，乌龟demo中的键盘控制话题是 /turtle1/cmd_vel，但ros提供了一个通用的键盘控制方案 /cmd_vel，使用下面的命令启动：
```shell
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

如果显示没有找到这个包的话：
```shell
	$ [rospack] Error: package ‘teleop_twist_keyboard‘ not found
```

安装一下就可以，但一定要安装对应版本的：
```shell
	$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```

还需要在当前文件夹下重新执行catkin_make，不用担心会将先前文件覆盖掉。

```shell
	$ catkin_make
```

当前有的话题：
```shell
	$ rostopic list
		/cmd_vel        # teleop_keyboard 发起的
		/rosout
		/rosout_agg
		/turtle1/cmd_vel  # turtle_keyboard 发起的
		/turtle1/color_sensor
		/turtle1/pose
```
现在的目标是将 /cmd_vel 或者 /turtle1/cmd_vel 进行重定向。
```launch
<!-- remap 重命名话题 -->
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen">
		<remap from="turtle1/cmd_vel" to="cmd_vel" />
	</node>
	
	<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>

</launch>
```

启动 teleop_twist_keyboard.py
```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
	Waiting for subscriber to connect to /cmd_vel
	
	Reading from the keyboard  and Publishing to Twist!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	
	For Holonomic mode (strafing), hold down the shift key:
	---------------------------
	   U    I    O
	   J    K    L
	   M    <    >
	
	t : up (+z)
	b : down (-z)
	
	anything else : stop
	
	q/z : increase/decrease max speeds by 10%
	w/x : increase/decrease only linear speed by 10%
	e/c : increase/decrease only angular speed by 10%
	
	CTRL-C to quit
	
	currently:	speed 0.5	turn 1.0 

```

------

param 标签

如果想要给节点传递参数，除了在\<node\>中添加 args 以外，还可以使用 param 标签。

方案一：将标签定义在node外
```launch
<launch>
	<param name="param_A" type="int" value="100" />
</launch>
```

方案二：将标签定义在node内
```launch
<launch>
	<node pkg="demo" type="launch_node" name="my_launch" output="screen">
		<param name="param_B" type="int" value="13" />
	</node>
</launch>
```

但是要注意，这两个方法的命名空间不同。
```shell
/my_launch/param_B    # 方案二

/param_A              # 方案一
```

-------

rosparam 标签

command：
* load：从文件中加载
* dump：保存到文件
* delete：从参数服务器中删除

使用yaml文件导入、导出参数。
yaml文件可以放在任何地方，通常情况下放在和launch同文件夹下。

```yaml
param_A: 100
param_B: 3.14
param_S: "zhang3"
```

方案一：将标签定义在node外
```launch
<launch>
	<rosparam command="load" file="${find demo}/launch/params.yaml" />
</launch>
```

方案二：将标签定义在node内
```launch
<launch>
	<node pkg="demo" type="launch_node" name="my_launch" output="screen">
		<rosparam command="load" file="$(find demo)/launch/params.yaml"/>
	</node>
</launch>
```

两个方案同样在命名空间上存在差异。
```shell
	$ rosparam list
		/my_launch/param_B
		/param_A
```

使用rosparam将参数从服务器中导出。
```launch
<rosparam command="dump" file="${find demo}/launch/params_out.yaml" />
```

【注】rosparam 标签优先级很高，会在node标签之前执行，如果想要把全部的参数都导出，先新建一个launch并将dump的操作放在新的launch文件中。
```launch
<launch>
	<rosparam command="dump" file=$(find demo)/launch/param_out.launch>
</launch>
```

从参数服务器中删除：
```launch
	<rosparam command="delete" param="param_A" />
```

---------

group  标签

对节点进行分组，特别是在node比较多的时候使用分组可以高效地管理这些节点。

```launch
<launch>

	<group ns="tutle_1">
		<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" />
		<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen" />
		<param name="turtle_1_param_A" type="int" value="123"/>
	</group>
	
	<group ns="tutle_2">
		<node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" />
		<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen" />
		<param name="turtle_1_param_A" type="int" value="456"/>
	</group>

</launch>
```
如果没有使用分组，那么在第二次启动 turtle_teleop_key 的时候，前一个启动的会被挤掉，同时注入的参数也会被更改。

现在有了group后再查看rosnode：
```shell
	$ rosnode list
		/rosout
		/tutle_1/my_key
		/tutle_1/my_turtle
		/tutle_2/my_key
		/tutle_2/my_turtle
```
可以发现相同的节点被分割成了不同的组。

而查看param可以发现，参数也被分到了不同的组：
```shell
$ rosparam list
	/rosdistro
	/roslaunch/uris/host_thinkpad__33435
	/roslaunch/uris/host_thinkpad__36137
	/rosversion
	/run_id
	/tutle_1/my_turtle/background_b
	/tutle_1/my_turtle/background_g
	/tutle_1/my_turtle/background_r
	/tutle_1/turtle_param_A
	/tutle_2/my_turtle/background_b
	/tutle_2/my_turtle/background_g
	/tutle_2/my_turtle/background_r
	/tutle_2/turtle_param_A
```

查看具体的参数值，可以发现，尽管参数同名了也没有发生覆盖：
```shell
	$ rosparam get /tutle_1/turtle_param_A
		123
	$ rosparam get /tutle_2/turtle_param_A
		456
```

---

arg 标签

动态传参，提升代码可用性。

当需要设置多个参数，但值相同的时候可以用arg，可以让以后修改参数值的时候不用全文去找，只需修改一个地方即可。

```launch
<launch>
	<arg name="car_length" default="0.55"/>
	<param name="A" value="$(arg car_length)" />
	<param name="B" value="$(arg car_length)" />
	<param name="C" value="$(arg car_length)" />
</launch>
```

参数结果
```shell
	$ rosparam list
		/A
		/B
		/C
```

此外还可以通过命令行的方式直接修改参数值：
```shell
	$ roslaunch demo arg_launch.launch car_length:=100
```
