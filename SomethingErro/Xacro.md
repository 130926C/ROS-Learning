## Xacro 

这个文件列出了在编写xacro过程中遇到的常见错误以及**可能的**解决方案。

由于xacro文件本身并不支持格式检查，需要检查的话要先转换成urdf文件：

```shell
$ rosrun xacro xacro car.xacro  > car.urdf
$ check_urdf car.urdf
```

用这种间接的方式来检查xacro书写是否合法。

------

### [ERROR] [1658653535.002229514]: radius [${footprint_r}] is not a valid float

这个错误类型有很多可能的原因：

#### 原因一：launch 文件错误

```xml
<launch>
    <param name="robot_description" textfile="$(find demo)/urdf/car.xacro"/>
</launch>
```

正确书写格式应该是用 command 来代替 textfile 属性，同时command属性还需要额外增加 **“$(find xacro)/xacro”** 部分。

```xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro $(find demo)/urdf/car.xacro"/>
</launch>
```

----------

### XML parsing error: mismatched tag: line 57, column 6

这个报错通常会出现在joint标签后面，即joint标签结束后出现了报错，检查joint标签是否多了一个 “/”，如：

```xml
53 	<joint name="wheel_joint_car_base" type="continuous" />
54 		<parent link="car_base" />
55		<child link="left_wheel" />
56		<origin xyz="0 0 ${wheel_bais_z}" rpy="1.57 0 0" />
57	</joint>
```

在上面的例子中，joint的一行就多了一个 “/” 导致joint提前被结束，所以他会报错在第57行。

------



