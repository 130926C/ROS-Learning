## ROS & OpenCV

这个demo是ROS和OpenCV结合的基本使用，调用本地的相机持续获取图像，然后sub对其进行色彩转换输出

在学习过程中发现并不需要像其他博客中写的卸载 ros-melodic-cv-bridge 来防止冲突，我是的ROS是严格按照[官方文档](http://wiki.ros.org/ROS/Installation)安装的，Opencv是下载源码进行编译的，以下是我的opencv版本。

```shell
    $ pkg-config --modversion opencv
    $ 3.2.0
```

这里主要参考了 **嵙杰** 的文章 [ROS-indigo发布Mat图像以及解决cv_bridge与OpenCV3.3.1版本不兼容问题](https://blog.csdn.net/lixujie666/article/details/80076083)

官方也有发布的参考代码：[Writing a Simple Image Publisher (C++)](http://wiki.ros.org/image_transport/Tutorials/PublishingImages)

在阅读网络上的其他博客时发现很多博客要在创建package的时候添加如下依赖：

```shell
    $ catkin_create_pkg image_package std_msgs rospy roscpp cv_bridge sensor_msgs image_transport
```

然而在开发过程中不太可能事先就知道需要哪些包和库，大多数情况都是后期用到什么再添加。ROS可以通过修改**package.xml**和**CMakeLists.txt**这两个文件来追加依赖，但这个方法有些麻烦。

下面的代码都是在仅仅包含了 roscpp、rospy、std_msgs 三个库后的操作，后面会修改上述两个文件以添加库。

---------

## 以下是错误示范，想看正确的方法直接跳到后面即可

刚开始想当然地觉得使用以下代码就可以将图像信息发布出去：
```cpp
int main(int argc, char *argv[]){
    ros::init(argc, argv, "load_image");
    ros::NodeHandle hd;

    ros::Publisher pub = hd.advertise<cv::Mat>("image_load", 10);
    cv::Mat img = cv::imread("/home/gaohao/Desktop/ROS-Learning/ROSandOpenCV/src/resource/test.jpeg");

    pub.publish(img);

    return 0;
}
```

实际上并不可以，会在 catkin_make 的过程中报以下错误:
```shell
$ error: ‘const class cv::Mat’ has no member named ‘__getMD5Sum’
```
这是因为ROS的OpenCV是不能直接调用的，需要进行转化成ROS规定的话题格式，所以不能这么写

-----

## 正确使用方式

这里不需要卸载安装任何库或功能，并且在创建包时没有添加关于opencv的包也可以

### **Publisher**

Step0:包含头文件
```cpp
    #include "ros/ros.h"
    #include "image_transport/image_transport.h"
    #include "opencv2/highgui/highgui.hpp"
    #include "cv_bridge/cv_bridge.h"
```

Step1:主函数 

使用ROS自带的转换函数将cv::Mat格式的数据转换
```cpp
int main( int argc, char **argv ){
    ros::init( argc, argv, "pub_cam_node" );
    ros::NodeHandle n;
 
    // 从本地读取摄像头
    cv::VideoCapture capture(0);
 
    // 【核心】创建opencv -> ROS message 格式的缓缓器 & 发布者
    image_transport::ImageTransport it( n ); 
    image_transport::Publisher pub_image = it.advertise( "camera", 1 );
 
    cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;
 
    while(ros::ok()) {
        capture >> frame->image;

    // 添加头信息
        frame->header.stamp = ros::Time::now();
        pub_image.publish( frame->toImageMsg() );
 
        cv::waitKey(3);
        ros::spinOnce();
    }
 
    capture.release();  //释放流
    return 0;
}
```

### **Subscriber**
Step2:主函数
```cpp
int main(int argc, char **argv){  
    ros::init(argc, argv, "image_sub_node");  
    ros::NodeHandle nh;  

    cv::namedWindow("view");  
    cv::startWindowThread();  

    image_transport::ImageTransport it(nh);  
    image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);  
    ros::spin();  
  
    cv::destroyWindow("view");  //窗口
}
```

Step3:回调函数 
```cpp
void imageCallback(const sensor_msgs::ImageConstPtr& msg){  
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);  
}  
```
【**注意**】尽管在 **Publisher** 文件中 publish 的数据类型为 **sensor_msgs::ImagePtr**，但在回调函数的位置上，需要使用 **const sensor_msgs::ImageConstPtr&**，否则在编译过程中会出现如下报错:
```shell
    $  error: could not convert ‘boost::forward<const boost::shared_ptr<const sensor_msgs::Image_<std::allocator<void> > >&>((* & a0))’ from ‘const boost::shared_ptr<const sensor_msgs::Image_<std::allocator<void> > >’ to ‘boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > >’
```
说明无法对发布和订阅的数据进行转化，因此这个坑一定要注意。

Step4:改写CMakeLists.txt文件  

不知道为什么，我发现即便不改写package.xml文件也能正常执行。

1. 在**find_package**中添加 sensor_msgs cv_bridge image_transport 并添加OpenCV的package
```txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
  sensor_msgs cv_bridge image_transport
)
find_package(OpenCV REQUIRED)
```

1. 在包含文件中添加OpenCV的头文件
```txt
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
```

3. 添加可执行文件
```txt
add_executable(pub_node src/camera_pub.cpp)
add_executable(sub_node src/camera_sub.cpp)
```

4. 添加依赖
```txt
add_dependencies(pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

5. 添加动态链接库
```txt
target_link_libraries(pub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(sub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```
【注意】这里的sub_node和pub_node必须分开写，写成这样报错：
```txt
target_link_libraries(pub_node sub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

```shell
$ CMake Error at camera_package/CMakeLists.txt:156 (target_link_libraries):
  Target "sub_node" of type EXECUTABLE may not be linked into another target.
```

