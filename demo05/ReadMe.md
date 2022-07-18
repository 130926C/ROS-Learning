### demo05
自定义参数服务，从参数服务器上增删改查

需要修改的文件有以下几个：
1.  c_cpp_properties.json
4. package.xml


Note:  
ROS的参数服务器是在roscore上的，所以无需手动维护一个服务器，并且在roscore运行时会被一直保留，可以通过 rosparam get 'name' 获得指定的参数

【注意】参数服务器上的参数任意一个ros节点都可进行修改，如果以“/params”开头的参数是全局参数，“/namespace1/params”的参数是在“/namespace1”作用域下的局部参数。
