cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/dynamic_params.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(config_node src/dynamicServer.cpp)
add_dependencies(config_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_node
  ${catkin_LIBRARIES}
)
