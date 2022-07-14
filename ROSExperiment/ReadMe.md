## ROS Experiment 

这个文件夹里是对学习过程中的一些课后题实现以及验证，因为刚开始学习的时候直接做了实验并没有留下任何记录，所以这部分内容可能无法和学习demoX保持同步，后期想起来多少会往里面添加。

---------

## Contents

**Exercise01**:这个实验验证了ROS topic在发布和订阅节点之间的数量关系，一个话题可以有多个发布者和多个订阅者，同时一个节点可以发布和订阅多个话题；

**Exercise02**:这个实验验证了ROS Server-Client 模式之间的数量关系，一个Server命题在当前是独占的，如果此时有另外一个相同命题Server B上线则会挤调前一个Server A，即便是Server B下线了，Server A也无法继续提供服务。

**Exercise03**:这个实验验证了ROS Server-Client 模式在多线程条件下的情况。



