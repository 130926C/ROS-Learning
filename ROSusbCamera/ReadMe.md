## ROS usb Camera

这个案例是一个很简单的ROS和usb相机的案例，使用ROS调用usb相机并输出得到图像的信息。这个案例应当作为 ROSandOpenCV 案例的先导。

-----

### **Step 1**：检查驱动 & 安装ROS包
如果你使用的是笔记本电脑，那么默认在ubuntu上是安装了摄像头驱动，如果使用的是第三方硬件可以使用下面的命令来检查是否成功安装上驱动：
```shell
$ ls /dev/v*
    /dev/video0
```
至少需要显示一个video0才能继续下面的步骤。

安装ROS的摄像头驱动：
```shell
$ sudo apt-get install ros-melodic-usb-cam
```

------

### **Step 2**：使用官方案例查看话题
使用launch文件开启摄像头
```shell
$ roslaunch usb_cam usb_cam-test.launch
```
查看当前话题
```shell
$ rostopic list
    /image_view/output
    /image_view/parameter_descriptions
    /image_view/parameter_updates
    /rosout
    /rosout_agg
    /usb_cam/camera_info
    /usb_cam/image_raw
    /usb_cam/image_raw/compressed
    /usb_cam/image_raw/compressed/parameter_descriptions
    /usb_cam/image_raw/compressed/parameter_updates
    /usb_cam/image_raw/compressedDepth
    /usb_cam/image_raw/compressedDepth/parameter_descriptions
    /usb_cam/image_raw/compressedDepth/parameter_updates
    /usb_cam/image_raw/theora
    /usb_cam/image_raw/theora/parameter_descriptions
    /usb_cam/image_raw/theora/parameter_updates
```
其中 /usb_cam/* 的话题都是摄像头发布的信息。 

-----

### **Stpe 3**：编写cpp文件

在创建功能包时添加以下依赖：
```txt
roscpp std_msgs sensor_msgs
```
**main.cpp**
```cpp
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void DealUSBCameraImage(
    const sensor_msgs::ImageConstPtr& imgP
){
    ROS_INFO("Received image [%d,%d] encoding=%s", imgP->width, imgP->height, imgP->encoding.c_str());
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle hd;

    ros::Subscriber sub = hd.subscribe("/usb_cam/image_raw", 10, DealUSBCameraImage);

    ros::spin();
    return 0;
}
```

修改 CMakeLists.txt 文件
```txt
add_executable(main_node src/main.cpp)

add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_node
  ${catkin_LIBRARIES}
)
```

----

### **Stpe 4**：执行launch和main_node 

【注意】roslaunch执行的时候会自动拉起roscore，因此不需要单独启动roscore，如果右面又拉起一个roscore会导致话题无法被订阅。

```shell
$ roslaunch usb_cam usb_cam-test.launch
```

```shell
$ rosrun demo main_node
    [ INFO] [1658490396.369897464]: Received image [640,480] encoding=rgb8
```
