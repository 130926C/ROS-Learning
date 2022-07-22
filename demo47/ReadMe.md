## demo 47

nodelet 官方案例实现。

在ROS中的通讯是基于Node结点的，一个node节点启动后会独占一个 **进程**，通讯本质上是进程之间的数据交换。

但是当数据比较大（图像、点云等）的时候，会出现延迟阻塞的情况。

解决方法是用Nodelet将多个节点**放在一个进程**中，这样算是线程间通讯，共享内存空间，使用**指针**来标注共享内存中的数据，代价要远低于进程间通讯。

Nodelet本质还是插件，只不过是对插件进行了封装。

* 提供了插件类的基类：Nodelet；
* 提供了加载插件的派生类加载器：NodeletLoader；

查看Nodelet语法：
```shell
$ rosrun nodelet nodelet 
    Your usage: 
    /opt/ros/melodic/lib/nodelet/nodelet 
    nodelet usage:
    nodelet load pkg/Type manager [--no-bond]  - Launch a nodelet of type pkg/Type on manager manager
    nodelet standalone pkg/Type   - Launch a nodelet of type pkg/Type in a standalone node
    nodelet unload name manager   - Unload a nodelet by name from manager
    nodelet manager               - Launch a nodelet manager node
```

----

### 使用Nodelet案例（命令行）
1. 启动nodeliet并设置一个经理
```shell
$ roscore
$ rosrun nodelet nodelet manager __name:=sun
    [ INFO] [1658469715.237527442]: Initializing nodelet with 20 worker threads.
```
启动了nodelet节点，并且将经理的名字命名为 sun 

2. 添加两个普通nodelet节点
```shell
$ rosrun nodelet nodelet load nodelet_tutorial_math/Plus sun __name:=wang _value:=100
    [ INFO] [1658472969.798243468]: Loading nodelet /wang of type nodelet_tutorial_math/Plus to manager sun with the following remappings:
```
往 sun 经理处添加一个名字为 wang，值为 100 的 nodelet_tutorial_math/Plus 节点。

```shell
$ rosrun nodelet nodelet load nodelet_tutorial_math/Plus sun __name:=zhang _value:=33
    [ INFO] [1658473516.888846792]: Loading nodelet /zhang of type nodelet_tutorial_math/Plus to manager sun with the following remappings:
```
往 sun 经历处再添加一个名字为 zhang，值为 33 的 nodelet_tutorial_math/Plus 节点。

3. 此时可以给 wang 和 zhang 两个节点发布消息：
```shell
查看现有话题:
$ rostopic list
    /rosout
    /rosout_agg
    /sun/bond
    /wang/in
    /wang/out
    /zhang/in
    /zhang/out
```

朝 /wang/in 发布消息:
```shell
$ rostopic pub -r 1 /wang/in std_msgs/Float64 "data: 15.7" 
```

查看 /wang/out 的输出信息：
```shell
$ rostopic echo /wang/out 
    data: 115.7
    ---
    data: 115.7
```

查看现有的参数服务器：
```shell
$ rosparam list
    /nodelet_tutorial_math_Plus/value
    /rosdistro
    /roslaunch/uris/host_gpuserver__39273
    /rosversion
    /run_id
    /wang/value
    /zhang/value
```

【**练习**】

让 wang 的输出是 zhang 的输入：
```shell
$ rosrun nodelet nodelet load nodelet_tutorial_math/Plus sun __name:=wang _value:=17 /wang/out:=/zhang/in       # wang 输出重定位

$ rosrun nodelet nodelet load nodelet_tutorial_math/Plus sun __name:=zhang _value:=100                          # zhang 正常启动
```

查看现有的话题可以发现 /wang/out 没有了，因为被重定向到 /zhang/in 了：
```shell
$ rostopic list
    /rosout
    /rosout_agg
    /sun/bond
    /wang/in
    /zhang/in
    /zhang/out
```

朝 /wang/in 发布话题 value=15 & 订阅 /zhang/out，预期因该是 15+17+100=132：
```shell
$ rostopic pub -r 1 /wang/in std_msgs/Float64 "data: 15.0"

$ rostopic echo /zhang/out 
    data: 132.0
    ---
    data: 147.0
```
发现一个现象：
* 当发布频率为 1Hz 的时候，/zhang/out 为 15+17+100=132 或 15+15+17+100=147
* 当发布频率为 10Hz 的时候，/zhang/out 仅为 15+17+100=132

造成这一现象的原因暂时没有找到，如果有大佬知道为什么请联系我。

-----

### 使用Nodelet案例（launch）

创建功能包 & 导入依赖
```txt
roscpp pluginlib nodelet
```

编写 run_nodelet.launch 文件：
```xml
<launch>
    <!-- 启动管理器 -->
    <node pkg="nodelet" type="nodelet" args="manager" name="sun" output="screen" />

    <!-- 启动wang-->
    <node pkg="nodelet" type="nodelet" args="load nodelet_tutorial_math/Plus sun" name="wang" output="screen">
        <param name="value" value="100" />
        <!-- 将 wang 的输出作为 zhang的输入 -->
        <remap from="/wang/out" to="/zhang/in" />
    </node>

    <!-- 启动zhang -->
    <node pkg="nodelet" type="nodelet" args="load nodelet_tutorial_math/Plus sun" name="zhang" output="screen" >
        <param name="value" value="67" />
    </node>

</launch>
```

【注意】这里有一个细节，如果想将节点A的输出作为节点B的输入，那么应该在节点A处处理，否则节点A都生成了就没法重定向了。
