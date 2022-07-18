### demo04
自定义服务通讯，求两个数的和并返回

需要修改的文件有以下几个：
1.  c_cpp_properties.json
2. tasks.json
3. settings.json
4. package.xml

需要添加的文件夹有以下几个：
1. src/demo/srv/AddNum.srv

【注意】在添加完服务后一定要先catkin_make 编译生成相关的头文件和源文件，如果后面想要修改srv中的内容，那么需要先将代码中出现的地方注释后再重新编译。