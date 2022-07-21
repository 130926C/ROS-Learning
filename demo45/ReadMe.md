## demo 45

动态参数服务器

进入 demo43 后执行下面的操作： 
```shell
$ roslaunch demo car.launch
$ roslaunch demo slam_auto.launch
$ rqt
```

然后在 rqt 的 GUI 界面中 “Plugins” -> “Congfiguration” -> "Dynamic Configure"；

在 “move_base_node” -> "global_costmap" -> "inflation_layer" 中可以调整全局地图的膨胀系数，这种动态的参数修改是实时的，不需要重启节点或者launch文件。

在创建 demo 功能包的时候需要添加以下依赖：
```txt
roscpp rospy std_msgs dynamic_reconfigure
```

【注意】因为 .cfg 文件本质是一个 python 文件，因此无论demo中有没有使用到python实现的客户端和服务端，都需要包含 rospy

----

### **Step 1**：创建cfg文件夹以及对应的文件

【Tips】因为 .cfg 文件本质是一个python文件，因此为了能够使用代码提示，可以先写成 .py，等代码写完了再改回 .cfg 文件。

**dynamic_param.cfg**
```python
#!/usr/bin python
from email.generator import Generator
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

# add: (name: Any, 
#       paramtype: Any, 
#       level: Any, 
#       description: Any, 
#       default: Any | None = None, 
#       min: Any | None = None, 
#       max: Any | None = None, 
#       edit_method: str = "") -> None
gen.add("int_param", int_t, 0, "a int param", 10, 1, 100)
gen.add("double_param", double_t, 0, "a double param", 0, 0, 3.14)
gen.add("string_param", str_t, 0, "a string param", "empty string")
gen.add("bool_param", bool_t, 0, "a bool param", True)


myList = gen.enum([gen.const("small", int_t, 0, "samll size"),
                   gen.const("middle", int_t, 1, "middle size"),
                   gen.const("big", int_t, 2, "big size")], 
                  "type value")

gen.add("list_param", int_t, 0, "choose list", 0, 0, 2, edit_method=myList)

# generate: (pkgname: Any, nodename: Any, name: Any) -> Any
exit(gen.generate("demo", "dr_client", "dynamic_param")) 
```

修改 CMakeLists.txt 文件
```txt
generate_dynamic_reconfigure_options(
  cfg/dynamic_param.cfg
)
```

编译生产中间件
```shell
$ catkin_make
```

-----

### **Step 2**：创建服务端

**server.cpp**
```cpp
#include "ros/ros.h"
#include "demo/dynamic_paramConfig.h"
#include "dynamic_reconfigure/server.h"

void CallbackFun(demo::dynamic_paramConfig &config, uint32_t level){
    ROS_INFO("after modify param is %d", config.int_param);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    dynamic_reconfigure::Server<demo::dynamic_paramConfig> server;
    // setCallback(const boost::function<void (demo::dynamic_paramConfig &, uint32_t level)
    server.setCallback(boost::bind(&CallbackFun, _1, _2));

    ros::spin();
    return 0;
}
```

-----

### **Step 3**：修改 CMakeLists.txt & 执行
```CMakeLists.txt
add_executable(server_node src/server.cpp)

add_dependencies(server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(server_node
  ${catkin_LIBRARIES}
)
```

```shell
$ roscore
$ rosrun demo server_node
$ rqt
```

进入rqt界面后 “Plugins” -> "Configuration" -> "Dynamic Reconfigure"，选择 “server_node” 即可实时修改 server_node 关注的动态参数。


