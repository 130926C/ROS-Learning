## Exercise02

这个实验测试了Server-Client模式的对应关系，得到以下结论：

1. 一个Server可以服务多个Client（大多数框架都这样）；
2. 一个Server可以提供多个服务；
3. 一个Client可以发出多种服务请求；
4. 一个Server命题只能被一个Server节点占用，后者会挤掉前者同时即便后者停止后前者也无法继续提供服务。**多个Server不能同时提供一个服务**



【注意】在ROS中的Server数据类型必须由srv文件和catkin_make指令生成，这是强制要求。即便是最简单的echo服务器也必须编写srv文件，因为ROS会生成对应的require与response对象，这些对象中有MD5加密函数。此外， **serviceClient**和**advertiseService**的回调都需要传入这种数据的引用。



