## Exercise 06

这个实验是用来验证ROS单个节点能否同时发布静态和动态坐标系关系，以及订阅节点如何处理从动态到静态坐标系的转化。

以下有几个注意事项：
* 从性能角度出发，如果静态坐标系是绝对静态（参考坐标系不是动态坐标系），那么最好优先发布
* 在发布动态坐标系的循环中应该使用的是 **ros::spinOnce()**，如果使用了 **ros::spin()** 会导致动态坐标系无法发布



关于如何选用  **ros::spinOnce()** 或者  **ros::spin()** ，CSDN博主 **肥肥胖胖是太阳** 的一篇文章写的很好 [ROS话题通信，最详细最易懂（从文件构建到执行原理）](https://blog.csdn.net/weixin_45590473/article/details/121208270) ，这个大佬的所有文章写的都非常好，推荐读者去看看。



总而言之，spin 系列的函数是用来激活回调函数的；在详细说明下面两个函数的区别之前，可以先将循环想象成一个展开的函数如：

```cpp
void CallbackFunc() {...}

while(i<3){
    i++;
    ros::spinOnce();	// ros::spin();
}
// 等价于下面三条语句
i++;
ros::spinOnce();		// ros::spin();		位置一
i++;
ros::spinOnce();		// ros::spin();		位置二
i++;
ros::spinOnce();		// ros::spin();		位置三
```



**ros::spin()** 会以阻塞的方式激活回调函数，程序会一直在回调函数那个位置等待新的请求，到达在位置一后就停驻在 **CallbackFunc** 的函数入口处等待其他请求；

**ros::spinOnce()** 以非阻塞的方式激活回调函数，程序到达位置一后会激活回调函数，然后让回调函数去忙，主循环继续到位置二；如果此时回调函数还没处理完，那么仍然会发送回调请求，主循环到位置三。。。；



所以为什么不能在循环中是使用  **ros::spin()** ，因为用了之后循环就无法继续执行了，会一直卡在回调函数那边；



当然 **ros::spinOnce()** 也不是完美的，因为是非阻塞的回调，所以可能会导致请求丢包，这样就需要设置好queue_size和处理频率，或者使用 **Exercise03** 中提到的方式，使用多线程的 spin。



-------

实验结果：
```cpp
$ rosrun demo sub_node 
    [ INFO] [1657946709.393127835]: "world" passed to lookupTransform argument target_frame does not exist. 
    [ INFO] [1657946710.391973427]: static_tf tf point [27.00, 35.00] in world tf is [37.00, 35.00]
    [ INFO] [1657946710.392074162]: dynamic_tf tf point [27.00, 35.00] in world tf is [38.00, 35.00]
    [ INFO] [1657946710.392105458]: static_tf tf point [27.00, 35.00] in dynamic_tf tf is [26.00, 35.00]
    [ INFO] [1657946710.392167545]: dynamic_tf tf point [27.00, 35.00] in static_tf tf is [28.00, 35.00]
```