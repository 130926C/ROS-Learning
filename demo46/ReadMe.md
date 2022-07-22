## demo 46

使用 ROS plugins 时需要在创建包的时候添加以下依赖：
```txt
roscpp pluginlib
```

------

### **Step 1**：添加 include 路径

在 c_cpp_properties.json 中添加 demo/include 路径，这一步只是为了在vscode下写代码的时候不给报错，并给出自动补全和提示，如果你的代码完全没问题也可以跳过这一步。
```json
"includePath": [
    "/opt/ros/melodic/include/**",
    "/usr/include/**",
    "/home/gaohao/Desktop/ROS-Learning/demo46/src/demo/include"
],
```

------

### **Step 2**：编写插件的基类
在 demo46/src/demo/include/demo 文件夹下编写插件的基类实现。

ROS要求插件的基类必须有一个无参构造。

**polygon_base.h**
```cpp
#ifndef __POLYGON_BASE_H__
#define __POLYGON_BASE_H__

namespace polygon_base_ns
{
    /*
        pluginlib 要求在基类中保留一个无参构造
    */
    class PolygonBase{
        protected:
            PolygonBase() {};
        public:
            virtual double getLength() = 0;
            virtual void init(double side_length) = 0;
    };
} // namespace polygon_base_ns

#endif 
```

------

### **Step 3**：编写插件派生类（实现类）
在基类文件的同目录下（推荐）实现插件的派生类

**polygon_Imp.h**
```cpp
#ifndef __POLYGON_IMP_H__
#define __POLYGON_IMP_H__

#include "polygon_base.h"

namespace polygon_base_ns
{
    class Triangle: public polygon_base_ns::PolygonBase{
        private:
            double side_length;
        public:
            Triangle(): side_length(0.0) {};
            double getLength(){ return 3.0 * this->side_length; }
            void init(double side_length) { this->side_length = side_length;}
    };

    class Square: public polygon_base_ns::PolygonBase{
        private:
            double side_length;
        public:
            Square(): side_length(0.0) {};
            double getLength() {return 4.0 * this->side_length; }
            void init(double side_length) {this->side_length = side_length; }
    };
} // namespace polygon_base_ns

#endif 
```

-----

### **Step 4**：单独写一个文件来注册插件
这个文件放在src目录下就行，也可以放在其他地方，记得改CMakeLists文件就可以。

**polygons.cpp**
```cpp
#include "pluginlib/class_list_macros.h"
#include "demo/polygon_base.h"
#include "demo/polygon_Imp.h"

PLUGINLIB_EXPORT_CLASS(polygon_base_ns::Triangle, polygon_base_ns::PolygonBase)
PLUGINLIB_EXPORT_CLASS(polygon_base_ns::Square, polygon_base_ns::PolygonBase)
```

-----

### **Step 5**：修改 CMakeLists.txt & 编译生成动态连接库 .so 文件

**CMakeLists.txt**
```txt
# 放开 inclucde
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# 添加自定义的动态连接库，并把这个连接库命名为 ploygonsPlugin
add_library(ploygonsPlugin
  src/polygons.cpp
)
```
【注意】动态连接库都是由cpp和h文件生成的，不是由xml这些直接生成的，因此必须由CMakeLists编译。

编译 & 生成so文件
```shell
$ catkin_make
    ...
    [100%] Linking CXX shared library /home/gaohao/Desktop/ROS-Learning/demo46/devel/lib/libploygonsPlugin.so
    [100%] Built target ploygonsPlugin
```

可以发现，构建工具在 devel.lib 文件夹下生成了 libploygonsPlugin.so，命名规则为: "lib" + 动态连接库名 + ".so"

-----

### **Step 6**：配置可用于ROS的工具链

在和package.xml同级目录下新建一个 polygon.xml 文件。

**polygon.xml**
```xml
<!--
    需要定位动态链接库 so
        library path 属性
    需要生命子类和父类
        class 
--> 

<library path="lib/libploygonsPlugin">
    <class type="polygon_base_ns::Triangle" base_class_type="polygon_base_ns::PolygonBase">
        <description>This is Triangle</description>
    </class>

    <class type="polygon_base_ns::Square" base_class_type="polygon_base_ns::PolygonBase" >
        <description>This is Square</description>
    </class>

</library>
```

----

### **Step 7**：修改package.xml文件
在export标签中添加Step6中编写的 polygon.xml 文件。

【注意】如果 polygon.xml 文件放在了其他地反记得修改文件路径。

**package.xml**
```xml
  <export>
    <demo plugin="${prefix}/polygon.xml" />
  </export>
```

为了检查插件是否成功配置上，可以在这个时候进行一次验证，使用 rospackage plugins 命令：
```shell
$ source devel/setup.bash
$ rospack plugins --attrib=plugin demo
demo /home/gaohao/Desktop/ROS-Learning/demo46/src/demo/polygon.xml
```
如果成功打印出文件路径那么说民插件配置成功，否则回头检查 **package.xml** 和 **polygon.xml** 这两个配置文件是否存在编写错误。

-----

### **Step 8**：使用插件

使用ROS的 plugins 插件有以下两步：
1. 创建插件类加载器；
2. 使用类加载器来创建插件实例；

在ROS中的插件都是用字符串进行描述的，在写的时候最好是先正常写，然后再用引号引成字符串。

**main.cpp**
```cpp
#include "ros/ros.h"
#include "demo/polygon_Imp.h"
#include "pluginlib/class_loader.h"
#include "demo/polygon_base.h"

int main(int argc, char *argv[]){
    
    // 创建类加载器
    pluginlib::ClassLoader<polygon_base_ns::PolygonBase> loader("demo", "polygon_base_ns::PolygonBase");

    try{
        // 使用加载器来实例化插件对象（一个基类加载器可以加载多个实例）
        boost::shared_ptr<polygon_base_ns::PolygonBase> triangle = loader.createInstance("polygon_base_ns::Triangle");
        boost::shared_ptr<polygon_base_ns::PolygonBase> square = loader.createInstance("polygon_base_ns::Square");

        // 使用
        triangle->init(10.0);
        square->init(50.2);
        ROS_INFO("Triangle length is %.2f", triangle->getLength());
        ROS_INFO("Square length is %.2f", square->getLength());
    }catch(std::exception& e){
        ROS_INFO(e.what());
    }

    return 0;
}
```

----

### **Step 9**：编译 & 执行
修改CMakeLists.txt
```makefile
add_executable(main_node src/main.cpp)

add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_node
  ${catkin_LIBRARIES}
)
```

执行
```shell
$ roscore
$ catkin_make
$ rosrun demo main_node
    [ INFO] [1658464550.412676502]: Triangle length is 30.00
    [ INFO] [1658464550.412698693]: Square length is 200.80
```
