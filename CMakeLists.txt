cmake_minimum_required(VERSION 2.8)
project(pcl_ros_conversion_benchmark)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  roscpp
  sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

catkin_package()

add_executable(pcl_ros_conversion_benchmark src/pcl_ros_conversion_benchmark.cpp)
target_link_libraries(pcl_ros_conversion_benchmark ${catkin_LIBRARIES} ${PCL_LIBRARIES})

