### demo11

获取当前时刻、指定时刻

在这个案例中，获得句柄操作是必须的，否则会导致时间相关的调用失败，先前部分案例特别是和参数服务器相关的案例有时候没有这一步操作，但在这里是必须有的。
```cpp
	ros::NodeHandle hd;
```
在这里的时间参考系是 1970年01月01日 00:00:00，所有的时间都是相对这个时间所流逝的时间。

-----

Experiment 1：获取当前距离参考时间过去了多少秒
```cpp
	ROS_INFO("Seconds: %.3f", right_now.toSec());
	ROS_INFO("Seconds: %d",   right_now.sec);

Output:
	[ INFO] [1650194419.136564304]: Seconds: 1650194419.137
	[ INFO] [1650194419.137616338]: Seconds: 1650194419
```

---

Experiment 2：设置一个绝对时间
```cpp
	ros::Time t1(53, 124131);
	ROS_INFO("Time: %.2f", t1.toSec());
	
Output:
	[ INFO] [1650194656.178024931]: Time: 53.00
```

------

Experiment 3：让程序休眠一段时间
```cpp
	ros::Duration dur(3.5);
	dur.sleep();
```

--------

Experiment 4：计算程序结束的时刻（利用时间的可加性、模拟耗时）
```cpp
// 方法一：时间的可加性

ros::Time ts(2.4);
ROS_INFO("Calc Start Time: %.2f; End Time: %.2f", ros::Time::now().toSec(), t1.toSec() + ts.toSec());


// 方法二：模拟
ros::Duration durs(ts.toSec());
t1 = ros::Time::now();
durs.sleep();

ROS_INFO("Simu Start Time: %.2f; End Time: %.2f", t1.toSec(), ros::Time::now().toSec());
```

---------

Experiment 5：定时器循环回调
```cpp
void TimerCb(const ros::TimerEvent& event){   // ros::TimerEvent 时间事件
	ROS_INFO("ROS Timer Callback");
	ROS_INFO("Current Time: %.2f", event.current_real.toSec());
}
```
也可以通过event来获取函数被调用的时刻。

```cpp
ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb);
ros::spin()
```

这里不使用 ros::Duration() 的原因是：这个函数只能用于休眠，需要配合结构体才能做到定时循环回调。使用 Timer 方法就可以让回调函数达到相同效果，有可能提升性能（因为没有额外的while判断语句）。

此外，回旋函数 ros::spin() 也是必须的，否则定时器并不会自发地调用。

------

Experiment 6：定时器只启动一次
```cpp
// oneshot=true
ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb, true);
```

---------

Experiment 7：定时器不自启动
```cpp
// autostart=false
ros::Timer timer = hd.createTimer(ros::Duration(1), TimerCb, false, false);

timer.start()
ros::spin()
```

-----

Add Experiment：获取前5秒的时刻

```cpp
ros::Time t1 = ros::Time() - ros::Duration(5);
```

