cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker_algorithm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/Walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})
