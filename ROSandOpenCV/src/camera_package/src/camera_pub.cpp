#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#define USE_CAMERA 1   // 发布视频流,否则发布图片

int main(int argc, char*argv[]){
  ros::init(argc, argv, "pub_cam_node");
  ros::NodeHandle hd;
 
  // 从本地读取摄像头
  #if USE_CAMERA
  cv::VideoCapture cap(0);
  #endif 

  // 创建opencv -> ROS message 格式的缓缓器 & 发布者
  image_transport::ImageTransport it(hd); 
  image_transport::Publisher pub_image = it.advertise("camera", 1);
 
  cv::Mat image;

  #if !USE_CAMERA
  cv::Mat image = cv::imread("/home/gaohao/Desktop/ROS-Learning/ROSandOpenCV/src/resource/test.jpeg");
  #endif

  cv_bridge::CvImage out_msg;

  while(ros::ok()) {
    #if USE_CAMERA
    cap >> image;
    #endif

    // 添加头信息
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;

    // 发布
    pub_image.publish(out_msg.toImageMsg());
 
    cv::waitKey(3);
    ros::spinOnce();
  }
 
  cap.release();  //释放流
  return 0;
}

