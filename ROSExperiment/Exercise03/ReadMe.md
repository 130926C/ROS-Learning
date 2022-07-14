## Exercise03

这个实验用来验证Server-Client在多线程下的状态，在先前的学习中可以发现，简单的Server-Client模式是阻塞响应的，即阻塞Server，对于后来的请求直接return false，实际上此时的client也是阻塞等待的，只不过对外表现不明显，因为client得到了false的返回就继续完成自己的事。和线程有关的[官方文档](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)。



如果某种请求十分重要但却又非常耗时，但线程的Server就无法提供可靠的服务，ROS也提供了一种解决方案，只需要将 **ros::spin()** 该写下就可以。



| spin方法                    | 阻塞   | 线程   |
| --------------------------- | ------ | ------ |
| ros::spin()                 | 阻塞   | 单线程 |
| ros::spinOnce()             | 非阻塞 | 单线程 |
| ros::MultiThreadedSpinner() | 阻塞   | 多线程 |
| ros::AsyncSpinner()         | 非阻塞 | 多线程 |



经过实验发现ROS提供的各种spin已经帮你实现了多线程操作，你只需要关心具体的处理逻辑即可：

-----

**ros::MultiThreadedSpinner()**

```shell
$	rosrun demo server_node 2
	Thread 140313613031168 processing 1 client ID
[ INFO] [1657769609.760190977]: Client ID=1, value=10
[ INFO] [1657769610.760323859]: Client ID=1, value=11
	Thread 140313596245760 processing 0 client ID
[ INFO] [1657769611.359765120]: Client ID=0, value=0
[ INFO] [1657769611.760275971]: Client ID=1, value=12
[ INFO] [1657769612.360002679]: Client ID=0, value=1
[ INFO] [1657769612.760395649]: Client ID=1, value=13
[ INFO] [1657769613.359876027]: Client ID=0, value=2
[ INFO] [1657769613.760260354]: Client ID=1, value=14
[ INFO] [1657769614.360002828]: Client ID=0, value=3
[ INFO] [1657769614.760361272]: Client ID=1, value=15
```

从上面可以发现处理两个client的请求线程分别是 **140313613031168** 和 **140313596245760**，说明ROS开辟了两个线程来处理。

---

这里出现了一个问题，按照官方文档使用**ros::AsyncSpinner**的server直接退出了，目前还没有找到解决方案，如果有大佬知道怎么解决麻烦给我留言，感激不尽。详细代码见 src/demo/server.cpp

