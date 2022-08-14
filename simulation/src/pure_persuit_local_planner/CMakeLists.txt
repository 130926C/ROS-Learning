cmake_minimum_required(VERSION 2.8.3)
project(pure_persuit_local_planner)

execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "x86_64" )
    set( CMAKE_CXX_FLAGS "-Wall -O3 -std=c++11" )
else()
    set( CMAKE_CXX_FLAGS "-Wall -O3 -std=c++11 " )
endif()

find_package(Boost)
find_package(catkin REQUIRED COMPONENTS roscpp  std_msgs sensor_msgs visualization_msgs 
                      geometry_msgs pluginlib nav_core tf costmap_2d base_local_planner)

catkin_package(LIBRARIES pure_persuit_local_planner 
	CATKIN_DEPENDS 
	pluginlib 
	roscpp
	nav_core
	base_local_planner
	costmap_2d
	std_msgs
    tf
)
include_directories(src
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/pure_persuit_local_planner.cpp
   src/Pid.cpp
 )

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

install(TARGETS
        ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   )
install(FILES pure_persuit_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


