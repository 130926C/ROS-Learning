## ROS与navigation教程-发布传感器数据

原文链接 [ROS与navigation教程-发布传感器数据](https://www.ncnynl.com/archives/201708/1886.html)

-----

使用下面代码发布虚假的激光雷达数据：
```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_scan_publisher");

    ros::NodeHandle n;
    // 激光雷达发布数据话题
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    int count = 0;
    ros::Rate r(1.0);
    while(n.ok()){
        //generate some fake data for our laser scan
        for(unsigned int i = 0; i < num_readings; ++i){
        ranges[i] = count;
        intensities[i] = 100 + count;
        }
        ros::Time scan_time = ros::Time::now();

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "laser_frame";   // 发布激光雷达数据的坐标系 laser_link
        scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        for(unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
        }

        scan_pub.publish(scan);
        ++count;
        r.sleep();
    }
}
```

-----

使用下面代码发布虚假点云数据：
```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_publisher");

    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

    unsigned int num_points = 100;

    int count = 0;
    ros::Rate r(1.0);
    while(n.ok()){
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        // 点云数据发布的坐标系名
        cloud.header.frame_id = "sensor_frame"; // livox_link

        cloud.points.resize(num_points);

        //we'll also add an intensity channel to the cloud
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[0].values.resize(num_points);

        //generate some fake data for our point cloud
        for(unsigned int i = 0; i < num_points; ++i){
        cloud.points[i].x = 1 + count;
        cloud.points[i].y = 2 + count;
        cloud.points[i].z = 3 + count;
        cloud.channels[0].values[i] = 100 + count;
        }

        cloud_pub.publish(cloud);
        ++count;
        r.sleep();
    }
}
```

