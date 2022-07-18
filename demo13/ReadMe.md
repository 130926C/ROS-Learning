### demo13

自定义的头文件要放入：demo/inclue 中；
和其对应的源文件要放入：demo/src 中；

这里的主要工作在于配置文件上：

文件配置上主要分为两步：
1. 配置源文件和头文件。
2. 配置主文件。

建议先配置完源文件和头文件相关内容后再按照惯性配置主文件，因为主文件配置和之前的完全一致。

【注意】配置自定义文件的时候不需要额外catkin_make，当然想试着输入下也是可以的，不会造成任何影响。

---------

Step 1：在 c_cpp_properties.json 中添加路径以增加提示。
```json
{
"configurations": [
{
"browse": {
"databaseFilename": "${workspaceFolder}/.vscode/browse.vc.db",
"limitSymbolsToIncludedHeaders": false
},

"includePath": [
"/opt/ros/noetic/include/**",
"/home/lucks/Desktop/code/ROS/demo13/src/demo/include/**",
"/usr/include/**"
],

"name": "ROS",
"intelliSenseMode": "gcc-x64",
"compilerPath": "/usr/bin/gcc",
"cStandard": "gnu11",
"cppStandard": "c++17"
}
],

"version": 4
}
```

---------

Step 2：放开include
```txt
include_directories(
include
${catkin_INCLUDE_DIRS}
)
```

----------

Step 3：声明C++库
```txt
add_library(head_src
src/${PROJECT_NAME}/demo.h
src/hello.cpp
)
```
声明要使用的头文件、源文件。将头文件和源文件打包成一个库，并命名为 "head_src"。

--------

Step 4：添加C++编译时依赖
```cpp
// 源文件与头文件
add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

// 主文件
add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```
让编译时依赖 head_src 这个库。

------

Step 5：添加C++运行时依赖
```cpp
// 主文件
target_link_libraries(main_node
head_src
${catkin_LIBRARIES}
)
```
让主文件运行时依赖 head_src 这个库。

在课程中还有下面这段：
```cpp
target_link_libraries(head_src
${catkin_LIBRARIES}
)
```
但是，根据上面的注释 "Specify libraries to link a library or executable target against" ，可以知道这部分的作用是给可执行文件指定动态链接库，他的操作对象是可执行文件而不是已经声明的动态链接库，所以不写这部分也可以运行起来。

-----------

Step 6：添加可执行文件（主文件）
```cpp
add_executable(main_node src/main.cpp)
```
【注】这个时候不用给源文件添加可执行文件，因为源文件里面没有main，此处添加了会出现“未定义main”的报错。

---------


关于python的自定义模块包导入：
```python
path = os.path.abspath(".")    #  获得当前解释器使用的绝对路径

os.path.insert(0, os.path.join(path, "src/demo/scripts"))   # 将这个路径插入到解释器中

import tools    # 导入自定义的包
```
这里有一个需要注意的点：由于python是解释性语言，所以每行的顺序会直接影响到结果，在这里就是需要先插入，然后再导包，否则还是会出现运行时报错。

