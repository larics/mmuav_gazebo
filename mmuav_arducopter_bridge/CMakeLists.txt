cmake_minimum_required(VERSION 2.8.3)
project(mmuav_arducopter_bridge)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

find_package(cmake_modules REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})