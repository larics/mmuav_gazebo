cmake_minimum_required(VERSION 2.8.3)
project(mmuav_description)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
  	rospy
  	roslib
)

catkin_package(
   INCLUDE_DIRS launch
#  LIBRARIES mmuav_description
#  CATKIN_DEPENDS gazebo roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)