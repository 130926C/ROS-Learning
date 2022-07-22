## demo 48

自定义 nodelet 通讯。

创建包时需要添加以下依赖：
```txt
roscpp pluginlib nodelet
```

----

### **Step 1**：创建一个集成于 nodelet 的派生类 
**myplus.cpp**
```cpp
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace nodelet_demo_ns {
class MyPlus: public nodelet::Nodelet {
    public:
    MyPlus(){
        value = 0.0;
    }
    void onInit(){
        //获取 NodeHandle
        ros::NodeHandle& nh = getPrivateNodeHandle();
        //从参数服务器获取参数
        nh.getParam("value",value);
        //创建发布与订阅对象
        pub = nh.advertise<std_msgs::Float64>("out",100);
        sub = nh.subscribe<std_msgs::Float64>("in",100,&MyPlus::doCb,this);

    }
    //回调函数
    void doCb(const std_msgs::Float64::ConstPtr& p){
        double num = p->data;
        //数据处理
        double result = num + value;
        std_msgs::Float64 r;
        r.data = result;
        //发布
        pub.publish(r);
    }
    private:
    ros::Publisher pub;
    ros::Subscriber sub;
    double value;

};
}

PLUGINLIB_EXPORT_CLASS(nodelet_demo_ns::MyPlus,nodelet::Nodelet)
```

----

### **Step 2**：修改CMakeLists.txt 并编译生产动态连接库
**CMakeLists.txt**
```txt
add_library(mynodeletlib
  src/myplus.cpp
)
...
target_link_libraries(mynodeletlib
  ${catkin_LIBRARIES}
)

```

----

### **Step 3**：生成ROS可用的工具链
**myplus.xml**
```xml
<library path="lib/libmynodeletlib">
    <class name="demo/MyPlus" type="nodelet_demo_ns::MyPlus" base_class_type="nodelet::Nodelet" >
        <description>hello</description>
    </class>
</library>
```

----

### **Step 4**：导出插件
**package.xml**
```xml
  <export>
      <nodelet plugin="${prefix}/myplus.xml" />
  </export>
```

到了这一步可以进行一次检查：
```shell
$ rospack plugins --attrib=plugin nodelet
    demo /home/gaohao/Desktop/ROS-Learning/demo48/src/demo/myplus.xml
    depth_image_proc /opt/ros/melodic/share/depth_image_proc/nodelet_plugins.xml
    stereo_image_proc /opt/ros/melodic/share/stereo_image_proc/nodelet_plugins.xml
    laser_filters /opt/ros/melodic/share/laser_filters/nodelets.xml
```
【注意】有时候自己写的 xml 文件会放在开头，有时候是末尾。

----

### **Step 5**：添加用于执行的 launch 文件
**run.launch**
```xml
<launch>
    <node pkg="nodelet" type="nodelet" name="my" args="manager" output="screen" />
    <node pkg="nodelet" type="nodelet" name="p1" args="load demo/MyPlus my" output="screen">
        <param name="value" value="100" />
        <remap from="/p1/out" to="con" />
    </node>
    <node pkg="nodelet" type="nodelet" name="p2" args="load demo/MyPlus my" output="screen">
        <param name="value" value="-50" />
        <remap from="/p2/in" to="con" />
    </node>
</launch>
```

