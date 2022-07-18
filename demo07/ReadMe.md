### demo07
实时控制乌龟运动，并且输出乌龟的Pose信息，同上一个demo一样，小乌龟的姿态信息也是通过话题的方式发布和动作的，只不过这次变成了订阅者，上一个demo是发布者。

通过下面的命令可以获得乌龟Pose的数据结构
```shell
$ rosmsg info turtlesim/Pose 
	float32 x
	float32 y
	float32 theta
	float32 linear_velocity
	float32 angular_velocity
```

这次需要添加的头文件是 **#include "turtlesim/Pose.h"**

这里还有一个小tips，在教程中订阅的输出速度是没有办法控制的，可以在回调函数中使用 sleep 让他等待一秒再输出。由于订阅的时候使用的是队列，所以不必担心这个等到无法获得实时信息，等待过程中产生的信息会被pop出队列。

```cpp
void dealMsg(const turtlesim::Pose::ConstPtr &pose){
	...
	sleep(1);
}
```
