cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo
#  CATKIN_DEPENDS roscpo rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_node src/publisher.cpp)
add_executable(sub_node src/subscriber.cpp)
add_executable(psub_node src/conpubsub.cpp)


add_dependencies(pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(psub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pub_node
  ${catkin_LIBRARIES}
)
target_link_libraries(sub_node
  ${catkin_LIBRARIES}
)
target_link_libraries(psub_node
  ${catkin_LIBRARIES}
)


