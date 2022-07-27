## demo 50

ROS 分布式配置。

ROS 在设计的伊始就考虑到分布式的需求，因此节点之间的通讯已经被ROS封装好了，没有特殊的需求最好不要自己实现节点之间的数据交互。

这个demo主要是展示如何配置 ROS 的主从机子。

【注意】在默认配置下主机和从机应该在同一个网络中。

-----

### **Step 1**：查询主从机IP地址

```shell
$ ifconfig | grep inet
```

假设这里的主从机 IP 地址分别为：
* 192.168.100.73        （主机） GPUServer
* 192.168.102.121       （从机） ThinkPadX1C

-----

### **Step 2**：配置主机
在ros中启动roscore的节点被称为 **主机**，并且要求当一套系统稳定运行的时候 **有且仅有一台** 主机，其余设备全部都为从机，通常情况下会将 **机器人控制节点**（工控机或TX2等嵌入式设备）定义成朱机，即产生数据的节点作为主机。

但这也不是绝对的，如果你的生产环境中存在多个产生数据的硬件，也可以将产生数据节点设置为从机，控制节点设置为主机，但仍要保证一个正在运行的系统中有且仅有一个主机节点。

1. 设置hosts文件
```shell
$ sudo gedit /etc/hosts
```
在文件末尾添加从机的地址，如果有多个从机就添加多个：
```txt
192.168.102.121 ThinkPadX1C
```

1. 编辑 ~/.bashrc 文件
```shell
export ROS_MASTER_URI=http://192.168.100.73:11311
export ROS_HOSTNAME=192.168.100.73
```
在文件两个export后面的值都是 **自己的** IP地址，后面的端口号 11311 是默认值，也可以自己修改，但要记得一会儿在从机的部分改掉这个号。

3. 让bashrc文件生效
```shell
$ source ~./bashrc
```

------------

### **Step 3**：配置从机
ROS中可以有多台从机，并且当 **主机启动后**，从机想要和主机建立联系的话 **不能启动 roscore**，否则相当于将这台从机独立出来了，和主机之间的联系也就中断了，会报出以下错误：
```shell
$ WARNING: ROS_MASTER_URI [http://192.168.100.73:11311] host is not set to this machine.
```

因此有以下两点：
* 主机roscore启动后，从机不能启动roscore；
* 从机也可以自行启动roscore，但这样就和主机失去联系；

【注意】并不是说将一台设备设置成从机后就再也不能启动roscore了，而是说启动后就无法和主机建立链接了。从机想要启动的话就变成了一个独立节点。

1. 设置 hosts 文件
```shell
$ sudo gedit /etc/hosts
```
在末尾添加主机地址，一个从机只能添加一个主机地址：
```txt
192.168.100.73 GPUServer
```

2. 编辑 ~/.bashrc 文件
```shell
$ sudo gedit ~/.bashrc
```
在末尾添加 **主机** 和 **从机** 的信息：
```shell
export ROS_NASTER_URI=http://192.168.100.73:11311   # 主机
export ROS_HOSTNAME=192.168.102.121                 # 从机
```

3. 让bashrc文件生效
```shell
$ source ~./bashrc
```

--------

### **Step 4**：测试连通性
在主机上：
```shell
$ ping ThinkPadX1C
$ ping 192.168.102.121
```

在从机上：
```shell
$ ping GPUServer
$ ping 192.168.100.73
```

四个ping都通了说明完全没有问题。如果ping不通，大概率是出现在 hosts 文件上写错了。

-----

### **Step 5**：在主机上启动 roscore
```shell
$ roscore
```

```shell
$ rosrun turtlesim turtlesim_node
```

-----

### **Step 6**：在从机上启动键盘控制节点
```shell
$ rosrun turtlesim turtle_teleop_key
Reading from keyboard
--------------------
Use arrow keys to move the turtle. 'q' to quit.
```
然后就可以在从机上控制主机上的乌龟运动了。

【注意】这里可能出现报错，如果报错了就去检查 .bashrc 文件写的对不对，特比是 export 后面那两个全大写的部分，这块很容易出错。