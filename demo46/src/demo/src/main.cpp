#include "ros/ros.h"
#include "demo/polygon_Imp.h"
#include "pluginlib/class_loader.h"
#include "demo/polygon_base.h"

int main(int argc, char *argv[]){
    
    // 创建类加载器
    pluginlib::ClassLoader<polygon_base_ns::PolygonBase> loader("demo", "polygon_base_ns::PolygonBase");

    try{
        // 使用加载器来实例化插件对象（一个基类加载器可以加载多个实例）
        boost::shared_ptr<polygon_base_ns::PolygonBase> triangle = loader.createInstance("polygon_base_ns::Triangle");
        boost::shared_ptr<polygon_base_ns::PolygonBase> square = loader.createInstance("polygon_base_ns::Square");

        // 使用
        triangle->init(10.0);
        square->init(50.2);
        ROS_INFO("Triangle length is %.2f", triangle->getLength());
        ROS_INFO("Square length is %.2f", square->getLength());
    }catch(std::exception& e){
        ROS_INFO(e.what());
    }

    return 0;
}
