### demo18

话题重名

期望的话题被覆盖 & 期望的话题没重名。

1. rosrun
2. launch 文件
3. 编码时期

----

**rosrun**
```shell
	$ rosrun turtlesim turtlesim_node
	$ rosrun turtlesim turtle_teleop_key

	$ rostopic list
		/turtle1/cmd_vel
		/turtle1/color_sensor
		/turtle1/pose
```


```shell
	$ rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel
	$ rostun teleop_twist_keyboard teleop_twist_keyboard.py 
```
【注】这里有一个地方需要留心，因为 turtlesim_node 是话题 /turtle1/cmd_vel 的订阅者，现在想要让输入从 turtle_teleop_key 中转向 teleop_twist_keyboard.py 上，那么需要改变的是 turtlesim_node 关注的话题名，而不是改变 turtle_teleop_key 发布的话题名，换而言之如果仅仅写了下面这段代码，那么是无效的：
```shell
	$ rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/cmd_vel
	$ rosrun turtlesim turtlesim_node
```
 上面这段代码的意思是将  turtle_teleop_key 发布的话题名改成 /cmd_vel，但 turtlesim_node 关注的仍然是 /turtle1/cmd_vel。但如果写的是下面这种也是有效的：
```shell
	$ rosrun turtlesim turtlesim_node
	$ rosrun turtlesim turtle_teleop_key
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel
```

如果想让这个乌龟GUI顺利收到键盘命令，那么需要在下面三个方案中任选其中一个：
1. 将 turtlesim_node 订阅的主题名改成 teleop_twist_keyboard.py 发布的主题名。
2. 将 teleop_twist_keyboard.py 发布的主题名改成 turtlesim_node 订阅的主题名。
3. 同时改变 turtlesim_node 和 teleop_twist_keyboard.py 订阅和发布的主题名为同一个。

----

**launch**

这种方法也有两种思路，一种是修改订阅方的话题名称，另一种是修改发布方的话题名称。这里只写修改订阅方的。
```launch
<!-- 用launch文件修改话题 -->
<launch>

	<node pkg="turtlesim" type="turtlesim_node" name="turtle_node" output="screen">
		<remap from="/turtle1/cmd_vel" to="/cmd_vel"/>
	</node>
	
	<node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="cmd_key"/>

</launch>
```

-----

**编码方式**

话题类型：
* 全局 /  【根目录】：写代码时需要以 “/” 开头；
* 相对 ergouzi：直接写；
* 私有 wangqiang：需要创建特殊的NodeHandle ("~")

```shell
	$ /ergouzi/wangqiang
```

```cpp
这种规定也适用于服务类型。

// A.全局话题：以 / 开头，此时和节点空间以及名字没有任何关系
ros::Publisher pub_g = hd.advertise<std_msgs::String>("/global_topic", 10);

// B.相对：直接写，此时和节点空间平级
ros::Publisher pub_r = hd.advertise<std_msgs::String>("relative_topic", 10);

// C.私有：需要创建特定 NodeHandle
ros::NodeHandle nd("~");
ros::Publisher pub_p = nd.advertise<std_msgs::String>("private_topic", 10);
```

```shell
	$ rostopic list
		/global_topic
		/relative_topic
		/topic_node/private_topic
```
全局话题和相对话题看上去一致，但一个是全局的一个是相对的。

但是，如果在上面写的时候写错了，将私有话题写成了全局话题：
```cpp
ros::NodeHandle nd("~");

ros::Publisher pub_p = nd.advertise<std_msgs::String>("/private_topic", 10);
```
那么这个话题会被提升为全局话题，即全局话题优先级更高。

```shell
	$ rostopic list
		/global_topic
		/private_topic
		/relative_topic
```

同理python实现如下：
```python
# A.全局
pub_g = rospy.Publisher("/topic_global",String, queue_size=10)

# B.相对
pub_r = rospy.Publisher("topic_relateive", String, queue_size=10)

# C.私有
pub_p = rospy.Publisher("~topic_private", String, queue_size=10)
```
