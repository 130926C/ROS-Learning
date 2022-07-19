## Exercise 05

多个动态坐标系之间相互转化。



这个demo和前一个 **Exercise04** 的内容非常相似，只不过一个是在静态坐标系上的，一个是在动态坐标系上，需要注意的是以下几点：

* 坐标系关系发布对象必须是 **tf2_ros::TransformListener**，不能再用静态的；
* 刷新点的时间戳  header.stamp  应该用 **ros::Time()**，而不能再用 **ros::Time::now()**，否则会报错，详细原因见下面描述；



在ROS中，坐标系关系监听对象有一个缓冲区，从坐标系发布者发布当前状态到存入缓冲区之间是有一定的时间的，因为在动态坐标系是指相对位置实时在变动的一个坐标系，所以时间的概念在这里非常重要，哪怕只差了0.1ms，动态坐标系的状态都可能发生改变。

如果**点**在请求坐标系关系时使用的是now()，可能会导致时间戳对不上，如下所示：

```shell
time		op
12:03:06	坐标系发布了自己和world之间的关系
12:03:07	点请求了坐标系转化关系操作
12:03:08	坐标系发布了自己和world之间的关系
```

从上面可以发现，如果点在 12:03:07 使用now()来请求转化计算，那么ROS会寻找时间戳为 12:03:07 的坐标系关系，然而实际上坐标系关系的时间戳是 12:03:06 和 12:03:08，没有一个符合要求，此时报错。因此在动态坐标系转化过程中应该使用 ros::Time() 让ROS找最近的一个时间戳计算，这样就不会报错了。

--------

而静态坐标系却没有这种问题，因为ROS在处理静态坐标系关系的时候使用的是两种完全不同的逻辑，静态坐标系在ROS中被认为是恒定不变的，时间戳只要摄制成在调用之前的就可以，因此在静态坐标系关系中，点可以使用 ros::Time::now() 来调用坐标系关系。