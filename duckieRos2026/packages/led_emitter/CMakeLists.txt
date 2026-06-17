cmake_minimum_required(VERSION 2.8.3)
project(led_emitter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    duckietown_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
