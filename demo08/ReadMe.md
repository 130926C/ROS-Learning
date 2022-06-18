### demo08
在固定位置上创建多个乌龟，代码层面其实不难，难的是如何找到对应的节点/服务/话题。在这里，这部分内容就不参考视频教程，用就一步一步试探的方式去找。

首先，一开始并不知道控制乌龟数量的是话题还是其他东西，那么从第一个能想到的话题开始找。
Step 1：查看当前所有的话题
```shell
$ rostopic list
	/rosout
	/rosout_agg
	/turtle1/cmd_vel
	/turtle1/color_sensor
	/turtle1/pose
```
从话题的输出上好像并没有特别明显的提示。 /cmd_vel 之前用过是拿来控制运动的（输入），pose是用来获得当前姿态的（输出）；而 /color_sensor 明显和实例不相关。

Step 2：尝试从节点入手
```shell
$ rosnode list
	/rosout
	/teleop_turtle
	/turtlesim

```
在节点中可以发现，最开始启动这个乌龟实例用的就是 /turtlesim 节点，而控制乌龟用的是 /teleop_turtle 节点。如果 /turtlesim 节点能够用来启动乌龟案例，那么其中应该包含了创建乌龟对象的操作，貌似看到了一点曙光。

Step 3：查看 /turtlesim 节点信息
```shell
$ rosnode  info /turtlesim 
----------------------------------------------------------
	Node [/turtlesim]
	Publications: 
	 * /rosout [rosgraph_msgs/Log]
	 * /turtle1/color_sensor [turtlesim/Color]
	 * /turtle1/pose [turtlesim/Pose]
	
	Subscriptions: 
	 * /turtle1/cmd_vel [geometry_msgs/Twist]
	
	Services: 
	 * /clear
	 * /kill
	 * /reset
	 * /spawn
	 * /turtle1/set_pen
	 * /turtle1/teleport_absolute
	 * /turtle1/teleport_relative
	 * /turtlesim/get_loggers
	 * /turtlesim/set_logger_level
	
	
	contacting node http://thinkpad:40851/ ...
	Pid: 21324
	Connections:
	 * topic: /rosout
	    * to: /rosout
	    * direction: outbound (36093 - 192.168.3.21:47318) [30]
	    * transport: TCPROS
	 * topic: /turtle1/cmd_vel
	    * to: /teleop_turtle (http://thinkpad:37371/)
	    * direction: inbound (43490 - thinkpad:40401) [32]
	    * transport: TCPROS
```
事实证明，想要学好计算机英文一定要好，要不然答案放在你面前都会错过，我因为对 “spawn” 这个单词不熟悉导致第一次就忽略了这个服务，直到后面走进死胡同后从头开始才整明白。

这个命令给出了 /turtlesim 节点的很多信息，其中 Publications 和 Subscriptions 表示这个节点发布和订阅了哪些话题，后面的 Services 表示节点开启了哪些服务。在这些服务中 **/spawn** 是 **产卵、引发、引起、导致、造成** 的意思，好像和我们期望的创造一个新乌龟很接近了，那就试试认为干预这个服务会有什么效果。

Step 4：查看 /spawn 服务详细信息。
```shell
$ rosservice info /spawn 
	Node: /turtlesim
	URI: rosrpc://thinkpad:36093
	Type: turtlesim/Spawn
	Args: x y theta name
```
虽然看上去貌似没有 “create” 这种直接的字样，但出现了 “name” 这个字段，那就试试用 call 方法对这个服务进行调用。

Step 5：使用call调用服务
```shell
$ rosservice call /spawn "x: 8.0
	y: 10.0
	theta: 0.0
	name: 'zhang3'" 
	name: "zhang3"
```
回头看小乌龟GUI窗口竟然成功了，这里创建了一个名为 “zhang3” 的小乌龟。再去查看这个节点信息。

Step 6：查看节点信息
```shell
$ rosnode info /turtlesim 
-------------------------------------------------------------------
	Node [/turtlesim]
	Publications: 
	 * /rosout [rosgraph_msgs/Log]
	 * /turtle1/color_sensor [turtlesim/Color]
	 * /turtle1/pose [turtlesim/Pose]
	 * /zhang3/color_sensor [turtlesim/Color]
	 * /zhang3/pose [turtlesim/Pose]
	
	Subscriptions: 
	 * /turtle1/cmd_vel [geometry_msgs/Twist]
	 * /zhang3/cmd_vel [unknown type]
	
	Services: 
	 * /clear
	 * /kill
	 * /reset
	 * /spawn
	 * /turtle1/set_pen
	 * /turtle1/teleport_absolute
	 * /turtle1/teleport_relative
	 * /turtlesim/get_loggers
	 * /turtlesim/set_logger_level
	 * /zhang3/set_pen
	 * /zhang3/teleport_absolute
	 * /zhang3/teleport_relative
```
发现在 Publications 和 Subscriptions 中都多了 /zhang3/... 的发布和订阅，为了验证能不能操控这只乌龟，可以进一步手动操作订阅信息。

Step 7：干预zhang3的订阅者
```shell
$ rostopic pub /zhang3/cmd_vel geometry_msgs/Twist "linear:
	  x: 10.0
	  y: 0.0
	  z: 0.0
	angular:
	  x: 0.0
	  y: 0.0
	  z: 1.0" 
	publishing and latching message. Press ctrl-C to terminate
```
乌龟它动了！说明这个思路是正确的，剩下就是代码实现。

