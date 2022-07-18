### demo17

对节点进行重命名

1. rosrun 
2. launch 文件
3. 编码时规定

想要一次启动两个 turtlesim_node 节点。

-----
**rosrun**

方法一：使用rosrun
```shell
	$ rosrun turtlesim turtlesim_node __ns:=zhang3
	$ rosrun turtlesim turtlesim_node __ns:=wang5
```

```shell
	$ rosnode list
		/rosout
		/wang5/turtlesim
		/zhang3/turtlesim
```

------

方法二：起别名
```shell
	$ rosrun turtlesim turtlesim_node __name:=li4
	$ rosrun turtlesim turtlesim_node __name:=zhao6
```

```shell
	$ rosnode list
		/li4
		/rosout
		/zhao6
```

方法三：叠加
```shell
	$ rosrun turtlesim turtlesim_node __ns:=ergouzi __name:=daqiang
	$ rosrun turtlesim turtlesim_node __ns:=maoluzi __name:=xiaoqiang	
```

```shell
	$ rosnode list
		/ergouzi/daqiang
		/maoluzi/xiaoqiang
		/rosout
```

--------

**launch**

方法一：名称重映射，修改name属性
```launch
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="tur_node" output="screen" />

	<node pkg="turtlesim" type="turtlesim_node" name="tur_node1" output="screen" />
</launch>
```

方法二：命名空间
```launch
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="tur_node" output="screen" ns="ergouzi"/>
</launch>
```

方法三：叠加
```launch
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="tur_node1" output="screen" ns="ergouzi"/>
</launch>
```

输出：
```shell
	$ rosnode list 
		/ergouzi/tur_node
		/ergouzi/tur_node1
		/rosout
		/tur_node
		/tur_node1
```

--------

