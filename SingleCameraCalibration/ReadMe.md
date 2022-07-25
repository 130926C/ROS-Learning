## Single Camera Calibration

单目相机标定。

单目相机的标定其实是很重要的一个内容，但是算法层面实现起来非常复杂，好在ROS提供了相机标定的功能包。

```shell
sudo apt-get install ros-melodic-camera-calibration
```

如果你是跟着《ROS 机器人开发技术基础》（蒋畅江等人著）的书学习的话，可能会遇到这么一个问题：

### **找不到 robot_vision 功能包**

并且发现全网好像也没有解决方案，唯一一个还是已经安装上了这个功能包后没有source的情况；同时这本书中好像也没有写这个包是从哪来的，怎么去安装这个包等等。

实际上，这个功能包是一个第三方包，而且是一个名不见经传的功能包，并不是ROS官方或者非常知名的第三方库。Github链接如下：
```xml
https://github.com/1027243334/robot_vision.git
```
--------

那么既然是第三方包的话，这里有两种方式安装它：
### **方法一**：在 catkin_ws 目录下安装
```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/1027243334/robot_vision.git
$ cd robot_vision
$ rosdep install -r --from-paths .
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE="Release"
```
这种安装方式有一个好处是以后想要使用的话就不用再下载了，直接source一下就可以使用：
```shell
$ source ~/catkin_ws/devel/setup.bash 
```

-------

### **方法二**：在当前包的src目录下添加这个包
```shell
$ cd src
$ git clone https://github.com/1027243334/robot_vision.git
$ catkin_make
$ source devel/setup.bash
```
这种方法的弊端在于每次使用都要重新下载一次，但是不会污染环境，避免以后有官方的同名包和这个冲突。在这个demo的src目录下中已经存放了这个 robot_vision 包，如果日后他的 Github 仓库关闭了还可以在这里拿一份备份。

------------

标定过程就比较简单了：

启动相机并开始采集图像
```shell
$ roslaunch robot_vision usb_cam.launch
```

打开标定程序
```shell
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
```
1. ”--size 8x6“ 的意思是你的标定板 **除掉最外一圈** 的黑白格后还剩下多少行和多少列；
2. “--square 0.024” 的意思是标定板单个格子的长度，单位m；
3. 后面的图像话题重定位就不多说了。

至于后面怎么操作可以看下面的链接：https://zhuanlan.zhihu.com/p/370996539

