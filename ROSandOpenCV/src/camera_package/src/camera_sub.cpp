#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg){  
  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);  
}  
 
int main(int argc, char **argv){  
    ros::init(argc, argv, "image_sub_node");  
    ros::NodeHandle nh;  

    cv::namedWindow("view");  
    cv::startWindowThread();  

    image_transport::ImageTransport it(nh);  
    image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);  
    ros::spin();  
  
    cv::destroyWindow("view");  //窗口
}