#include "ros/ros.h"
#include "demo/dynamic_paramConfig.h"
#include "dynamic_reconfigure/server.h"

void CallbackFun(demo::dynamic_paramConfig &config, uint32_t level){
    ROS_INFO("after modify param is %d, %.2f, %s, %d, %d", 
        config.int_param,
        config.double_param,
        config.string_param.c_str(),
        config.bool_param,
        config.list_param
    );
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "server_node");
    ros::NodeHandle hd;

    dynamic_reconfigure::Server<demo::dynamic_paramConfig> server;
    // setCallback(const boost::function<void (demo::dynamic_paramConfig &, uint32_t level)
    server.setCallback(boost::bind(&CallbackFun, _1, _2));

    ros::spin();
    return 0;
}
