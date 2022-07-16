## ROS Experiment 

这个文件夹里是对学习过程中的一些课后题实现以及验证，因为刚开始学习的时候直接做了实验并没有留下任何记录，所以这部分内容可能无法和学习demoX保持同步，后期想起来多少会往里面添加。

---------

## Contents

**Exercise01**:这个实验验证了ROS topic在发布和订阅节点之间的数量关系，一个话题可以有多个发布者和多个订阅者，同时一个节点可以发布和订阅多个话题；

**Exercise02**:这个实验验证了ROS Server-Client 模式之间的数量关系，一个Server命题在当前是独占的，如果此时有另外一个相同命题Server B上线则会挤调前一个Server A，即便是Server B下线了，Server A也无法继续提供服务。

**Exercise03**:这个实验验证了ROS Server-Client 模式在多线程条件下的情况。

**Exercise04**:这个实验测试了ROS中多个静态坐标系下的随机点位置变化，静态的含义是坐标系静态，而不是点是静态的，同时发现了ROS中的几个坑点，特别是 **StaticTransformBroadcaster** 和 **TransformBroadcaster** 不可乱用。

**Exercise05**:这个实验测试了ROS中多个动态坐标系下的随机点位置变化，和静态转化不同的是，在调用转化计算的时候不能使用 **ros::Time::now()** 而应该使用 **ros::Time()**让ROS自己找最近的时间戳。

**Exercise06**:这个实验测试了单个节点发布动态和静态两种坐标系关系，以及订阅节点如何转化动态和静态坐标系上的点。

**Exercise07**:这个实验测试了
