### demo14

多个功能包管理 metapackage。

在这个demo中有两个已经编写好的功能包，分别是 服务-客户端的ros_cs 和 发布-订阅的ros_pub。

这个demo展示了如何将这两个功能包封装进一个虚拟包中并调用。

1. 创建一个新的元功能包，不需要添加任何依赖。
2. 修改 package.xml
3. 修改 CMakeLists.txt

-----

Step 1：创建一个虚拟包在同级目录下
```cpp
src/
	ros_cs         // 客户端-服务端功能包
	ros_pub        // 发布者-订阅者功能包 
	ros_merage     // 虚拟包
```
【注】由于这个功能包并没有实际功能，在ros中相当于一个“目录”的作用，所以在 Create Catkin Package 的时候不需要添加任何依赖，只用输入包名即可。

------

Step 2：修改 package.xml 文件，添加可执行依赖
```xml
<buildtool_depend>catkin</buildtool_depend>

<exec_depend>ros_cs</exec_depend>
<exec_depend>ros_pub</exec_depend>
```
这一步将想要包含进来的功能包写上，让这个虚拟包在执行时从这两个包中找。

-----

Step 3：在 export 中声明一个空标签
```xml
<export>
<!-- Other tools can request additional information be placed here -->
	<metapackage />
</export>
```

-------

Step 4：修改 CMakeLists.txt
```txt
cmake_minimum_required(VERSION 3.0.2)
project(ros_merage)
find_package(catkin REQUIRED)
catkin_metapackage()
```
1. 只保留到 find_package及其以上行，其他全部删除。
2. 添加：catkin_metapackage()

【注】在CMakeLists中不可以有换行，否则可能导致编译失败。

-----


以 ROS 自带的 navigation 功能包为例，官网链接如下：
ROS Navigation: http://wiki.ros.org/cn/navigation  
Github : https://github.com/ros-planning/navigation  

在这个功能包中，进入 navigation 子包可以看见只有声明没有实现：
```shell
	CHANGELOG.rst
	CMakeLists.txt
	README.rst
	package.xml
```
说明这个包也是封装了ros的其他包来实现导航功能。

查看里面的 package.xml 文件可以发现
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
    <name>navigation</name>
    <version>1.17.1</version>
    <description>
        A 2D navigation stack that takes in information from odometry, sensor
        streams, and a goal pose and outputs safe velocity commands that are sent
        to a mobile base.
    </description>
    <maintainer email="mfergs7@gmail.com">Michael Ferguson</maintainer>
    <maintainer email="davidvlu@gmail.com">David V. Lu!!</maintainer>
    <maintainer email="ahoy@fetchrobotics.com">Aaron Hoy</maintainer>
    <author>contradict@gmail.com</author>
    <author>Eitan Marder-Eppstein</author>
    <license>BSD,LGPL,LGPL (amcl)</license>  
    <url>http://wiki.ros.org/navigation</url>

    <buildtool_depend>catkin</buildtool_depend>

    <exec_depend>amcl</exec_depend>
    <exec_depend>base_local_planner</exec_depend>
    <exec_depend>carrot_planner</exec_depend>
    <exec_depend>clear_costmap_recovery</exec_depend>
    <exec_depend>costmap_2d</exec_depend>
    <exec_depend>dwa_local_planner</exec_depend>
    <exec_depend>fake_localization</exec_depend>
    <exec_depend>global_planner</exec_depend>
    <exec_depend>map_server</exec_depend>
    <exec_depend>move_base</exec_depend>
    <exec_depend>move_base_msgs</exec_depend>
    <exec_depend>move_slow_and_clear</exec_depend>
    <exec_depend>navfn</exec_depend>
    <exec_depend>nav_core</exec_depend>
    <exec_depend>rotate_recovery</exec_depend>
    <exec_depend>voxel_grid</exec_depend>

    <export>
        <metapackage/>
    </export>
</package>
```

exec_depend 是 navigation 依赖的功能包，而export里面放了一个空的 metapackage 标签。以后写ros的元功能包可以按照这个方式写。

他的 CMakeLists.txt 文件也只有四行。
```txt
cmake_minimum_required(VERSION 3.0.2)
project(navigation)
find_package(catkin REQUIRED)
catkin_metapackage()
```

【注】只用写这四行代码，不要添加或删除其他代码，否则可能编译报错。

