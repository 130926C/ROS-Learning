## Exercise 06

这个实验是用来验证ROS单个节点能否同时发布静态和动态坐标系关系，以及订阅节点如何处理从动态到静态坐标系的转化。

以下有几个注意事项：
* 从性能角度出发，如果静态坐标系是绝对静态（参考坐标系不是动态坐标系），那么最好优先发布
* 在发布动态坐标系的循环中应该使用的是 **ros::spinOnce()**，如果使用了 **ros::spin()** 会导致动态坐标系无法发布

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