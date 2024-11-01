cmake_minimum_required(VERSION 3.5)
project(groundObstacle)

set(CMAKE_CXX_STANDARD 17)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy        
        sensor_msgs
        std_msgs
)

catkin_package()
include_directories(tools ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)

set(tool_source
        tools/tool.cpp
        tools/tool.h
        tools/nanoflann.hpp)

add_executable(detectGroundObstacle detect/main.cpp ${tool_source})
target_link_libraries(detectGroundObstacle
        ${catkin_LIBRARIES} ${PCL_LIBRARIES})        

# tool-merge PCs
add_executable(mergePC tools/mergePC.cpp ${tool_source})
target_link_libraries(mergePC
        ${catkin_LIBRARIES} ${PCL_LIBRARIES})