### demo02
简单话题通讯，在demo01基础上，对传输的内容添加一个变量。

需要修改的文件有以下几个：  
1. c_cpp_properties.json
2. tasks.json

Note:  
在C++实现中，如果使用了std::stringstream作为传输信息的变量，那么一定要确保在循环中调用了 .str("") 这个函数，否则会导致上一次的消息没有被清理，传入的是累积的信息。 实际上最好还是用sprintf的方式来组织字符串：

```cpp
char str[100];
sprintf(str, "messgae_%d", i);
```

以这种方式组织的字符串不用担心被累积的情况。
