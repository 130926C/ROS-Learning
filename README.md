# ROS Self-Learning

因工作需要使用到ROS，所以写了这个仓库作为学习日志。第一次尝试写这类日志还有很多不足的地方，如果大佬发现了那块存在问题可以直接和我的邮箱联系。343905080@qq.com

## Description

学习资料主要来源于以下内容：
* [深蓝学院](https://www.shenlanxueyuan.com/my/course/92)
* [B站赵虚左的视频课程](https://www.bilibili.com/video/BV1Ci4y1L7ZZ?spm_id_from=333.337.search-card.all.click);
* [ROS wiki 官方学习资料](http://wiki.ros.org/ROS/Tutorials);


除了上述学习资料外，还推荐以下三本参考书：
* 《ROS机器人开发技术基础》，蒋畅江、罗云翔、张宇航等编著，化学工业出版社；
* 《精通ROS机器人编程》，Lentin Joseph等编著，机械工业出版社；
* 《视觉SLAM十四讲-从理论到实践》，高翔、张涛等著，电子工业出版社；

关于这个仓库的文件夹名字描述
* demoX:是每一个案例的文件夹；
* SomethingError:是在学习过程中遇到的一些离奇报错以及应对方案；
* ROSandXXX:是将ROS和其他库结合起来的demo，主要是机器视觉这一大块；
* ROSExperiment:是对上面教材的课后题以及一些实验验证；

## Notes
这里使用的是vscode作为代码编写环境，然而由于vscode会生成.vscode文件夹，这个文件夹中的文件内容会根据本机的环境会生成不同的内容。

如果想要参考的话最好先读每一个demo中的ReadMe文件，里面会写一些前期流程。

在安装ROS-noetic的时候，遇到任何报错都应该当场解决，否则后面会无法推进。

【注】在demo01～demo29的时候使用的都是ros-noetic，后面由于工作需要改成了ros-melodic，但大体内容不受影响，主要的变动的是在 .vscode 文件夹下的 c_cpp_properties.json 部分，并且有且仅有添加 inlcude 文件路径时会有变动。

如果你使用的也是vscode的话需要安装以下插件：
* C/C++
* C/C++ Extension Pack
* ROS (这个非常重要，因为可以直接右键创建ros的package，不用再使用命令行创建功能包)

每一个demo无论是否使用到代码实现节点（ROS可以直接使用launch文件启动节点）都执行了以下操作来确保整体风格

```shell
  $ mkdir -p demoX/src
  $ cd demoX
  $ catkin_make
```

这里使用到的 .gitignore 

```txt
*.bin
devel/
build/
*.db
*.vc.db-shm
*.vc.db-wal
```
-----
## How to Use
Step1:安装git
```shell
$ sudo apt-get install git
```
Step2:克隆这个仓库
```shell
$ git clone git@github.com:130926C/ROS-Learning.git
```
Step3:随便进入一个目录
```shell
$ cd ROS-Learning/demo01
```
Step4:在当前目录下编译
```shell
$ catkin_make
```
编译结束后会生成build、devel、.catkin_workspace文件，这些文件的具体使用和功能将在后面的demo中介绍。

-----

## Contents

[**demo01**](demo01):简单话题通讯，发布者订阅者模式

[**demo02**](demo02):在demo01的基础上对发布在话题上的信息添加了变量

[**demo03**](demo03):自定义话题通讯，通讯的内容使用msg文件指定

[**demo04**](demo04):自定义服务，求两个数的和，使用srv文件定义服务元素

[**demo05**](demo05):自定义参数服务器，在参数服务器上增删该查

[**demo06**](demo06):使用话题订阅模式控制小乌龟运动

[**demo07**](demo07):在小乌龟运动的时候以订阅的方式输出Pose信息

[**demo08**](demo08):创建多个乌龟

[**demo09**](demo09):修改参数服务器上的参数，从而改变小乌龟对话框的背景颜色

[**demo10**](demo10):节点复用 & 在启动节点的时候给参数服务器设置值

[**demo11**](demo11):获取ROS中的当前时刻以及指定时间

[**demo12**](demo12):ROS日志信息，INFO、DEBUG、WARN、ERROR、FATAL

[**demo13**](demo13):使用自定义的.h和.cpp文件

[**demo14**](demo14):metapackage多个功能包管理

[**demo15**](demo15):用launch文件启动小乌龟

[**demo16**](demo16):关于不同工作空间内的节点重名

[**demo17**](demo17):用命令行和launch文件的方式实现节点复用

[**demo18**](demo18):对话题进行重命名

[**demo19**](demo19):参数服务器上的全局与私有参数

[**demo20**](demo20):分布式ROS通讯

[**demo21**](demo21):TF静态坐标变化(话题订阅)

[**demo22**](demo22):Rviz和动态坐标变化(话题订阅)

[**demo23**](demo23):多个静态坐标系变化(launch & 话题订阅)

[**demo24**](demo24):使用坐标信息实现小乌龟跟踪

[**demo25**](demo25):rosbag录制操作并回放

[**demo26**](demo26):使用rqt工具箱以及rqt-plot绘制小乌龟pose信息

[**demo27**](demo27):URDF机器人建模，在rviz中显示一个正方体

[**demo28**](demo28):URDF组合机器人，使用link和joint

[**demo29**](demo29):使用xacro代替URDF构建机器人

[**demo30**](demo30):使用xacro的include方式组装小车

[**demo31**](demo31):使用Actionlib实现双向Server-Client通讯

[**demo32**](demo32):欧拉角、四元数、旋转矩阵

[**demo33**](demo33):GUI界面的动态参数服务器、参数发布与订阅

[**demo34**](demo34):用arbotix控制小车在rviz中运动

[**demo35**](demo35):在gazebo中添加一个长方体

[**demo36**](demo36):给小车添加碰撞检测和惯性矩阵，并在gazebo中显示

[**demo37**](demo37):在gazebo中搭建自己的模型【这个非常吃电脑性能，虚拟机慎重】

[**demo38**](demo38):URDF、Gazebo、Rviz综合使用

[**demo39**](demo39):Gazebo多种传感器（雷达、摄像头、深度相机）仿真

[**demo40**](demo40):仿真环境下的机器人导航（SLAM建图）

[**demo41**](demo41):仿真环境下的机器人导航（acml定位）

[**demo42**](demo42):仿真环境下的机器人导航（move_base路径规划）

[**demo43**](demo43):仿真环境下的机器人导航（SLAM自动建图与导航）

[**demo44**](demo44):Action通讯

[**ROSExperiment**](ROSExperiment):存放了一些关于上面demo的衍生实验

[**ROSandOpenCV**](ROSandOpenCV):ROS和OpenCV的结合使用，发布订阅视频流和单个图片

[**SomethingErro**](SomthingErro):记录了一些常见的bug，包括编译和运行过程中的
