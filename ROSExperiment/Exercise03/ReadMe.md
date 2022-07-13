## Exercise03

这个实验用来验证Server-Client在多线程下的状态，在先前的学习中可以发现，简单的Server-Client模式是阻塞响应的，即阻塞Server，对于后来的请求直接return false，实际上此时的client也是阻塞等待的，只不过对外表现不明显，因为client得到了false的返回就继续完成自己的事。



如果某种请求十分重要但却又非常耗时，但线程的Server就无法提供可靠的服务，ROS也提供了一种解决方案，只需要将 **ros::spin()** 该写下就可以。



| spin方法                    | 阻塞   | 线程   |
| --------------------------- | ------ | ------ |
| ros::spin()                 | 阻塞   | 单线程 |
| ros::spinOnce()             | 非阻塞 | 单线程 |
| ros::MultiThreadedSpinner() | 阻塞   | 多线程 |
| ros::AsyncSpinner()         | 非阻塞 | 多线程 |



