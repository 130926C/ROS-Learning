### demo31 

使用 **Arbotix** 控制小车运动。

1. 创建机器人urdf、xacro文件。
2. 添加 Arobtix 配置文件。
3. 在 launch 文件中配置 Arobtix。
4. 启动 launch 文件。

安装 Arbotix：
```shell
$ sudo apt-get install ros-noetic-arbotix
```
如果出现下面提示，说明版本太新还没有提供可用的包：
```shell
$ E: Unable to locate package ros-noetic-arbotix
```
需要从源码中编译：

