## Exercise01

这个实验验证了ROS 的话题topic关于发布者和订阅者之间的数量关系：

1. 一个话题可以被多个节点订阅；
2. 一个节点可以发布多个话题；
3. 一个话题可以被多个节点发布；
4. 一个节点可以同时是发布者和订阅者；

以下几个文件完成了实验：
* conpubsub.cpp：一个节点同时是发布者和订阅者，将一个话题的信息订阅后转发到另一个话题上；
* publisher.cpp：可复用的发布者
* subscriber.cpp：可复用的订阅者 


【注意】：在ROS中，topic发布的数据类型是需要 **严格** 遵守ROS的数据规则，如果直接按下面的方式写是无法通过编译的：

```cpp
    ros::Publisher pub = hd.advertise<std::string>("topic", 10);
```

这种写法在语法中不会报错，但 catkin_make 编译的时候是会出现类型错误。

