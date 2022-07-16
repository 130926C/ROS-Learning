#include "ros/ros.h"
#include "demo/dynamic_paramsConfig.h"
#include "dynamic_reconfigure/server.h"

void RefreshDynamicParams(demo::dynamic_paramsConfig& config, uint32_t params_level){
    ROS_INFO("Params changed, curr params is [%d, %.2f, %s, %d]", 
        config.int_param,
        config.double_param,
        config.string_param.c_str(),
        config.size
    );
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "config_node");
    ros::NodeHandle hd;

    dynamic_reconfigure::Server<demo::dynamic_paramsConfig> DynamicParamsServer;
    dynamic_reconfigure::Server<demo::dynamic_paramsConfig>::CallbackType CallbackType;

    CallbackType = boost::bind(&RefreshDynamicParams, _1, _2);
    DynamicParamsServer.setCallback(CallbackType);

    ros::spin();
    return 0;
}
