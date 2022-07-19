## demo 37

创建自己的 world 文件。

有来两种方式可以创建仿真环境：
1. 使用gazebo内置的组建搭建仿真环境；
2. 手动绘制仿真环境；

【注意】这个demo非常吃计算机性能，如果是虚拟机的话慎重使用，我这边使用的是物理机，非常流畅，CPU利用率一直在10%附近，配置如下：
* CPU:  i7-12700K
* GPU:  Nvidia-3060
* Mem:  64 GB
* OS:   Ubuntu 18.04
* Disk: Intel SSD
------

### 方法一：
启动gazebo，然后在上方的工具栏中拖动模型。
```shell
$ roscore 
$ rosrun gazebo_ros gazebo
```

最后 “File” -> “Save World As” 保存创建好的环境。

【画外音】：

官方提供了很多gazebo的资源库：https://github.com/osrf/gazebo_models.git，整个文件大概 **1.3 GB**，下载比较耗时。

下载好后需要使用命令的方式将 gazebo_models 中 **文件夹** 复制过去：
```shell
$ cd  /usr/share/gazebo-9/models
$ sudo cp -r /home/gaohao/Download/gazebo_models/* .
```
【注意】这里的gazebo-9/models文件夹下必须直接放模型文件夹，如果直接把gazebo_models文件夹移进去是不行的，在界面中是无法读取的。拷贝完成之后需要重启 gazebo。

-----------

### 方法二：
启动gazebo。/
```shell
$ roscore 
$ rosrun gazebo_ros gazebo
```

在 “Edit” -> "Building Editor" 中拖动和搭建各种构建，结束后 “File” -> “Save As”。

-----