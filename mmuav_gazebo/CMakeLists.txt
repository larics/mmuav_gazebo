cmake_minimum_required(VERSION 2.8.3)
project(mmuav_gazebo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
)

find_package(cmake_modules REQUIRED)


catkin_package(
  INCLUDE_DIRS launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)