## ROS与navigation教程-编写自定义全局路径规划

原文链接 [ROS与navigation教程-编写自定义全局路径规划](https://www.ncnynl.com/archives/201708/1887.html)

自己编写的路径规划器必须作为插件添加到 ROS 中，这样才可以被 move_base 使用。

----

### 自定义全局规划器头文件。

global_planner.h
```cpp
/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

// 使用命名空间防止冲突 
namespace global_planner {
    // 全局规划器需要继承自 nav_core::BaseGlobalPlanner
    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:

    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                );
    };
};

#endif
```

global_planner.cpp
```cpp
#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

// 必须使用这个宏指令将注册规划器展开
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner,nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

    GlobalPlanner::GlobalPlanner (){
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        plan.push_back(start);
        for (int i=0; i<20; i++){
                geometry_msgs::PoseStamped new_goal = goal;
                tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

                new_goal.pose.position.x = -2.5+(0.05*i);
                new_goal.pose.position.y = -3.5+(0.05*i);

                new_goal.pose.orientation.x = goal_quat.x();
                new_goal.pose.orientation.y = goal_quat.y();
                new_goal.pose.orientation.z = goal_quat.z();
                new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }
        plan.push_back(goal);
        return true;
    }
};
```

在 CMakeLists.txt 中添加以下代码：
```txt
add_library(global_planner_lib src/path_planner/global_planner/global_planner.cpp)
```

-----

### 添加插件描述文件

global_planner_plugin.xml
```xml
<library path="lib/libglobal_planner_lib">
    <class name="global_planner/GlobalPlanner" type="global_planner::GlobalPlanner" base_class_type="nav_core::BaseGlobalPlanner">
        <description>This is a global planner plugin by iroboapp project.</description>
    </class>
</library>
```

-----

### 注册插件的 ROS 包中
在 package.xml 文件中添加以下内容
```xml
<build_depend>nav_core</build_depend>
<run_depend>nav_core</run_depend>

<export>
    <nav_core plugin="${prefix}/global_planner_plugin.xml" />
</export>
```

-----

### 查看插件是否成功添加
```shell
$ rospack plugins --attrib=plugin nav_core
```

-----

### 添加到 move_base
move_base.launch

```xml
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_global_planner" value="global_planner/GlobalPlanner"/>
</node>
```

----

### 启动导航与定位文件

auto_move.launch
```xml
<launch>
    <include file="$(find turtlebot_bringup)/launch/minimal.launch"></include>

    <include file="$(find turtlebot_bringup)/launch/3dsensor.launch">
        <arg name="rgb_processing" value="false" />
        <arg name="depth_registration" value="false" />
        <arg name="depth_processing" value="false" />
        <arg name="scan_topic" value="/scan" />
    </include>

    <!-- Map server -->
    <arg name="map_file" default="your_map_folder/your_map_file.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- Localization -->
    <arg name="initial_pose_x" default="0.0"/>
    <arg name="initial_pose_y" default="0.0"/>
    <arg name="initial_pose_a" default="0.0"/>
    <include file="$(find turtlebot_navigation)/launch/includes/amcl.launch.xml">
        <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
        <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
        <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>
    </include>

    <!-- Move base -->
    <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml"/>
</launch>
```

-----

### 实际操练习

在ROS空间下clone [ros-planning/navigation_tutorials](https://github.com/ros-planning/navigation_tutorials) 仓库

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-planning/navigation_tutorials
$ cd ..
$ catkin_make

$ roscore
```

**发布激光雷达数据**
```shell
$ rosrun laser_scan_publisher_tutorial laser_scan_publisher
```

**发布里程计数据**
```shell
$ rosrun odometry_publisher_tutorial odometry_publisher
```

**发布点云数据**
```shell
$ rosrun point_cloud_publisher_tutorial point_cloud_publisher
```

**发布机器人坐标系**
```shell
$ rosrun robot_setup_tf_tutorial tf_broadcaster
```

**订阅机器人坐标系**
```shell
$ rosrun robot_setup_tf_tutorial tf_listener
```

**修改导航终点**
```shell
$ rosed simple_navigation_goals_tutorial simple_navigation_goals.cpp 

goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.2;
```

**启动导航**
```shell
$ rosrun simple_navigation_goals_tutorial simple_navigation_goals
```