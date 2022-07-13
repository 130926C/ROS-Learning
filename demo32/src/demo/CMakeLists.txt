cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(E2Q_node src/Euler2Quaternion.cpp)
add_executable(Q2E_node src/Quaternion2Euler.cpp)

add_dependencies(E2Q_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Q2E_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(E2Q_node
  ${catkin_LIBRARIES}
)
target_link_libraries(Q2E_node
  ${catkin_LIBRARIES}
)
