## ROS and Object Detection

使用 ROS 调用笔记本摄像头实现目标检测。

这里需要安装ROS提供的第三方库 **find_object_2d**

```shell
$ sudo apt-get install ros-melodic-find-object-2d
```

实际上基于图像的二维物体检测在深度学习中已经非常常见了，后面会有demo实现ROS和 Yolo 模型的目标检测。

----

### **Step 1**：使用 robot_vision 开启摄像头
```shell
$ roslaunch robor_vision usb_cam.launch
```

### **Step 2**：将 find_object_2d 包上的功能节点数据重定向到 usb_cam 采集到的话题上 
```shell
$ rosrun find_object_2d find_object_2d image:=/usb_cam/image_raw
    [ INFO] [1658829679.003232390]: gui=1
    [ INFO] [1658829679.004178739]: objects_path=
    [ INFO] [1658829679.004209040]: session_path=
    [ INFO] [1658829679.004233351]: settings_path=/home/gaohao/.ros/find_object_2d.ini
    [ INFO] [1658829679.004274323]: subscribe_depth = false
    [ INFO] [1658829679.006883409]: object_prefix = object
    [ INFO] [1658829679.006920925]: pnp = true
```
【注意】这里的 find_object_2d 功能包可能无法直接Tab补齐，但是只要先敲进去，后面的 find_object_2d 是可以自动补齐的。

----

【注意】这里也存在一个问题，传统的目标检测算法非常吃性能，这会导致显示帧率在笔记本上只有两三帧，后面的demo使用到了深度学习模型会稍微改善一些，但只在CPU上跑的话最好别抱太大的期望。
