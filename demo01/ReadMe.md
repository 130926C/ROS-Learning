### demo01
简单话题通讯，只使用了固定内容的字符串进行通讯。

需要修改的文件有以下几个：  
1. c_cpp_properties.json
2. tasks.json

Note:  
如果发布者只发布了一次消息但并没有使用 ros::Duration(1)的话，在terminal中使用 rostopic echo hello 很可能无法捕捉到消息。