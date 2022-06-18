#include "demo/hello.h"
#include "ros/ros.h"

namespace hello_ns{
    void MyHello::run(){
        ROS_INFO("Hello NS namespace run() function");
    }
};
