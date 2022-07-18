### demo09
使用修改背景颜色。

仍然可以用demo08的方法尝试自己找有关颜色的话题/服务等信息。

Step 1：查看节点 & 该节点包含的内容
```shell
$ rosnode list
$ rosnode info /turtlesim

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
```
输出中只有一个订阅的话题是和颜色相关的 /turtle1/color_sensor，可以先手动试下发布这个内容。

Step 2：手动发布话题信息
```shell
$ rostopic pub /turtle1/color_sensor turtlesim/Color "r: 0
	g: 255
	b: 0" 
```
发现界面没有反映，说明背景颜色并不是通过话题订阅的方式决定的。那么继续去找可能的方式。除了话题以外有可能改变背景颜色的就是服务，但在Step 1中可以发现并没有和颜色相关的服务。那么就剩下最后一种可能——参数服务器。

Step 3：查看参数服务器中保存的参数
```shell
$ rosparam list
	/rosdistro
	/roslaunch/uris/host_thinkpad__34735
	/rosversion
	/run_id
	/turtlesim/background_b
	/turtlesim/background_g
	/turtlesim/background_r
```
好像发现了答案，在参数服务器中有三个以 background 相关的参数名，那就试下手动改变这三个变量。

Step 4：手动改变参数服务器中的参数
```shell
$ rosparam set /turtlesim/background_r 150
$ rosservice call /clear "{}"   # 刷新服务
```
设置完参数后需要刷新一下服务，否则背景颜色也不会改变。

手动方式可行后就可以进行代码层面的实现，此外还添加了动态刷新的部分。下面将详细说明如何实现这个功能。

能明确的是刷新的命令是：
```shell
$ rosservice call /clear "{}"
```
既然用了 rosservice call 那么说明这是 server-client模型，可以使用这个方法来等价命令行操作。

Step 1：先获得 /clear 的数据类型
```shell
$ rosservice info /clear 
	Node: /turtlesim
	URI: rosrpc://thinkpad:56757
	Type: std_srvs/Empty
	Args: 
```
发现这个的数据类型是 std_srvs/Empty，那么C++代码的头文件就必须包含这个。

Setp 2：获得当前这个server维护的服务
```shell
$ rosservice list
	/clear
	/kill
	/reset
```
发现这服务就叫 /clear，那么就直接发送这个请求给 server 就可以了，或者重启小乌龟的GUI界面。具体代码实现在文件中可见。

