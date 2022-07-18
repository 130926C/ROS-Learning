### demo03
自定义话题通讯，通讯的内容是自定义的结构体

需要修改的文件有以下几个：
1.  c_cpp_properties.json
2. tasks.json
3. settings.json
4. package.xml

需要添加的文件夹有以下几个：
1. src/demo/msg/Person.msg

【注意】：为了能够在vscode中出现代码提升，修改c_cpp_properties.json的include是必须的，但由于这里用的是catkin_make，所以即便不修改也不会在编译期间报错，最多就是写代码不方便。
