cmake_minimum_required(VERSION 2.8.3)
project(mission_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    duckietown_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
