## ROS and QR recognition

ROS 和二维码识别。

ROS已经封装好了一个二维码识别库 ar_track_alvar，但还是需要先安装一下：
```shell
$ sudo apt-get install ros-melodic-ar-track-alvar
```

这个包提供了很多功能，包括生成、识别等，这个demo先生产一个二维码然后再去识别它。

-----

### **Step 1**：生成二维码
生成二维码需要使用到 ar_track_alvar 包下的 createMarker 节点，查看这个节点提供的参数列表可以发现这个可以生成很多类型的二维码：
```shell
$ rosrun ar_track_alvar createMarker 
    SampleMarkerCreator
    ===================

    Description:
    This is an example of how to use the 'MarkerData' and 'MarkerArtoolkit'
    classes to generate marker images. This application can be used to
    generate markers and multimarker setups that can be used with
    SampleMarkerDetector and SampleMultiMarker.

    Usage:
    /opt/ros/melodic/lib/ar_track_alvar/createMarker [options] argument

        65535             marker with number 65535
        -f 65535          force hamming(8,4) encoding
        -1 "hello world"  marker with string
        -2 catalog.xml    marker with file reference
        -3 www.vtt.fi     marker with URL
        -u 96             use units corresponding to 1.0 unit per 96 pixels
        -uin              use inches as units (assuming 96 dpi)
        -ucm              use cm's as units (assuming 96 dpi) <default>
        -s 5.0            use marker size 5.0x5.0 units (default 9.0x9.0)
        -r 5              marker content resolution -- 0 uses default
        -m 2.0            marker margin resolution -- 0 uses default
        -a                use ArToolkit style matrix markers
        -p                prompt marker placements interactively from the user


    Prompt marker placements interactively
    units: 1 cm 0.393701 inches
    marker side: 9 units
    marker id (use -1 to end) [0]: 
```

其中 marker id 是二维码的编号，这个是生成的时候必须提供的，如果想要生成一个 baidu.com 的链接二维码可以这样写：
```shell
$ rosrun ar_track_alvar createMarker 0 -3 www.baidu.com
    ADDING MARKER 0
    ADDING MARKER www.baidu.com
    Saving: MarkerData_0_www_baidu.png
```
这个会在当前目录下生成一个名字为 **MarkerData_0_www_baidu.png** 的二维码，然后我发现手机扫不出来，只能用他这个库来识别。

-----

### **Step 2**：修改图像发布话题

从 ar_track_alvar 包中将用来识别的文件夹拷贝过来然后修改相关话题。
```shell
$ roscd ar_track_alvar
$ sudo cp pr2_indiv_no_kinect.launch  /home/gaohao/Desktop/ROS-Learning/ROSandQRRecognition/src/demo/launch/
```

**pr2_indiv_no_kinect.launch**
```xml
<launch>
	<arg name="marker_size" default="4.4" />
	<arg name="max_new_marker_error" default="0.08" />
	<arg name="max_track_error" default="0.2" />
	<!-- 修改成自己发布的图像话题 -->
	<arg name="cam_image_topic" default="/usb_cam/image_raw" />
	<!-- 修改成自己发布的标定参数话题 -->
	<arg name="cam_info_topic" default="/usb_cam/camera_info" />
	<!-- 修改成图像所在的坐标系 -->
	<arg name="output_frame" default="/usb_cam" />

	<node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
		<param name="marker_size"           type="double" value="$(arg marker_size)" />
		<param name="max_new_marker_error"  type="double" value="$(arg max_new_marker_error)" />
		<param name="max_track_error"       type="double" value="$(arg max_track_error)" />
		<param name="output_frame"          type="string" value="$(arg output_frame)" />

		<remap from="camera_image"  to="$(arg cam_image_topic)" />
		<remap from="camera_info"   to="$(arg cam_info_topic)" />
	</node>
</launch>
```

----

### **Step 3**：启动launch文件
```shell
$ roslaunch robot_vision usb_cam_with_calibration.launch
$ roslaunch demo pr2_indiv_no_kinect.launch 
```

识别出来的二维码信息会被发布到 **/ar_pose_marker** 话题中