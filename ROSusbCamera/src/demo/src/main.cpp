#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void DealUSBCameraImage(
    const sensor_msgs::ImageConstPtr& imgP
){
    ROS_INFO("Received image [%d,%d] encoding=%s", imgP->width, imgP->height, imgP->encoding.c_str());
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle hd;

    ros::Subscriber sub = hd.subscribe("/usb_cam/image_raw", 10, DealUSBCameraImage);

    ros::spin();
    return 0;
}
