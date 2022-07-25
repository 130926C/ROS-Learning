## demo33
带有GUI界面的动态参数服务器配置、参数发布与订阅

在使用ROS的时候有些参数是会发生变化的，但是参数服务器不会主动告知节点参数发生了变化，所以需要使用到动态参数服务器。



使用动态参数最重要的意义在于 **可以不用重启节点就能更新参数**，这个在当前的阶段可能体会不大，但在后期时有很强的作用。

先前的demo中使用参数服务器更新了小乌龟界面的背景颜色，但是颜色的显示并不是实时刷新的，需要额外调用一个服务"/reset"或"/clear"才能刷新完成，或者关闭小乌龟GUI界面再重启。这是参数服务器最大的问题，为了解决这个问题，ROS提供了动态参数。

---------



使用动态参数和自定义msg、srv一样都需要声明文件并编译获得.h和.cpp文件。

【注意】动态参数服务器需要使用到python解释器，所以在新建package的时候要带上 rospy

**Step 1**：在src目录下创建cfg文件夹，并创建一个文件用来存放动态参数 dynamic_params.cfg

```shell
$ tree
    .
    ├── cfg
    │   └── dynamic_params.cfg
    ├── CMakeLists.txt
    ├── include
    │   └── demo
    ├── package.xml
    └── src
```

因为.cfg文件不是vscode能代码补全的文件类型，所以可以先将其修改成 .py 结尾的文件，写完后再改回来。

-------

**Step 2**：向 dynamic_params.cfg 文件中写入需要动态参数服务器管理的参数

```py
from dynamic_reconfigure.parameter_generator_catkin import *

# 创建参数生成器
gen = ParameterGenerator()

# 添加参数 
gen.add("int_param", int_t, 0, "int param", 10, 0, 20)
gen.add("double_param", double_t, 0, "double param", 5.0, 0.0, 10.0)
gen.add("string_param", str_t, 0, "string param", "empty str")

size_num = gen.enum([
    gen.const("Low", int_t, 0, "Low=0"),
    gen.const("Medium", int_t, 1, "Medium=1"),
    gen.const("High", int_t, 2, "High=2")],
    "select from the list"    
)

gen.add("size", int_t, 0, "select size", 1, 0, 3, edit_method=size_enum)

# 推出并生成中间件
exit(gen.generate("demo", "config_node", "dynamic_config"))
```

【注意】在最后一句话中，第一个变量是当前包名，第二个变量是参数服务器节点名，第三个变量是这个不带后缀的文件名。

其中第二个变量就是CMakeLists.txt文件中add_executable语句内填充的名字。

为了防止在执行 exit(gen.generate("demo", "config_node", "dynamic_params")) 时报错，先在 src 目录下新建一个空的 dynamicServer.cpp 文件，这样也方便配置CMakeLists.txt文件。

```cpp
#include "ros/ros.h"

int main(int argc, char *argv[]){
    
    return 0;
}
```

--------

**Step 3**：修改CMakeList.txt 和 package.xml 文件

CMakeLists.txt

```txt
cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/dynamic_params.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(config_node src/dynamicServer.cpp)
add_dependencies(config_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_node
  ${catkin_LIBRARIES}
)
```

package.xml

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>dynamic_reconfigure</build_depend>
  <build_depend>message_generation</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>dynamic_reconfigure</build_export_depend>


  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>dynamic_reconfigure</exec_depend>
  <exec_depend>message_runtime</exec_depend>
```

------

**Step 4**：编译生成中间件 & 并添加vscode路径

```shell
$ catkin_make
```

./vscode/c_cpp_properties.json

```json
      "includePath": [
        "/opt/ros/melodic/include/**",
        "/usr/include/**",
        "/home/gaohao/Desktop/ROS-Learning/demo33/devel/include"
      ]
```

-----

**Step 5**：编写动态参数服务器 dynamicServe.cpp

```cpp
#include "ros/ros.h"
#include "demo/dynamic_paramsConfig.h"
#include "dynamic_reconfigure/server.h"

void RefreshDynamicParams(demo::dynamic_paramsConfig& config, uint32_t params_level){
    ROS_INFO("Params changed, curr params is [%d, %.2f, %s, %d]", 
        config.int_param,
        config.double_param,
        config.string_param.c_str(),
        config.size
    );
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "config_node");
    ros::NodeHandle hd;

    dynamic_reconfigure::Server<demo::dynamic_paramsConfig> DynamicParamsServer;
    dynamic_reconfigure::Server<demo::dynamic_paramsConfig>::CallbackType CallbackType;

    CallbackType = boost::bind(&RefreshDynamicParams, _1, _2);
    DynamicParamsServer.setCallback(CallbackType);

    ros::spin();
    return 0;
}

```

-------

**Step 6**：启动节点 & 调用 GUI 界面修改参数

```shell
$ rosrun demo config_node
```

```shell
$ rosrun rqt_reconfigure rqt_reconfigure 
```

```shell
$ rosrun demo config_node 
[ INFO] [1657968484.385411646]: Params changed, curr params is [10, 5.00, empty str, 1]
[ INFO] [1657968523.877338988]: Params changed, curr params is [16, 2.20, hell ros, 1]
```

可以发现，由于使用了 ros::spin() 的方式，server一直阻塞等待参数的变化。查看当前参数服务器上的值：

```shell
$ rosparam list
/config_node/double_param
/config_node/int_param
/config_node/size
/config_node/string_param
/rosdistro

$ rosparam get /config_node/int_param 
16
```



