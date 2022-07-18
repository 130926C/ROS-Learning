### demo20

ROS分布式通讯

------

Step 1：双方添加IP地址
修改 /etc/hosts 文件，将对方的 ip地址加入，主机名称可以用 hostname 获取。

```shell
	$ hostname
		thinkpad
```

-------

Step 2：让host文件生效
最保险的方式就是重启，然后用ping检查链接情况。

```shell
	$ ping IP_address or hostname
```

-------

Step 3：设置主机和从机

主机：在 ~/.bashrc 中添加
```shell
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=主机IP
```

从机：在 ~/.bashrc 中添加
```shell
export ROS_MASTER_URI=http://主机IP：11311
export ROS_HOSTNAME=从机IP
```

IP地址后面的端口是可以随意指定的，但主从机上的这个数字一定要相同。

最后都要source一下：
```shell
	$ source .bashrc
```

----

Step 4：在主机上启动roscore
```shell
	$ roscore
```
【注】roscore只能在主机上启动，由主机控制ros系统。

-------

Step 5：在主机和从机上启动各自的任务节点
```shell
	$ rosrun turtlesim turtlesim_node (主机)
```

```shell
	$ rosrun turtlesim turtle_teleop_key  (从机)
```

