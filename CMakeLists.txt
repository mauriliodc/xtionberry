cmake_minimum_required(VERSION 2.8.3)
project(xtionberry)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS "-O3")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(~/src/OpenNI2/Include)
include_directories(~/src/OpenNI2/Samples/Common/)
include_directories(~/src/OpenNI2/Include/Linux-Arm)
link_directories(/home/pi/src/OpenNI2/Bin/Arm-Release)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xtionberry
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xtionberry_node src/xtionberry_node.cpp)
target_link_libraries(xtionberry_node
   ${catkin_LIBRARIES}
   rt OpenNI2
 )

