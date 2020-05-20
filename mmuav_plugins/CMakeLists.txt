cmake_minimum_required(VERSION 3.5.2)
project(mmuav_plugins)


add_definitions(-std=c++11)


# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  mav_msgs
  rosbag
  roscpp
  rotors_comm
  rotors_control
  std_srvs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES mmuav_gazebo_ductedfan_motor_model
  CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs rosbag roscpp rotors_comm rotors_control std_srvs tf
  DEPENDS eigen3 gazebo opencv
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_library(mmuav_gazebo_ductedfan_motor_model src/gazebo_ductedfan_motor_model.cpp)
target_link_libraries(mmuav_gazebo_ductedfan_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(mmuav_gazebo_ductedfan_motor_model ${catkin_EXPORTED_TARGETS})

add_library(mmuav_gazebo_gimbal src/gazebo_gimbal.cpp)
target_link_libraries(mmuav_gazebo_gimbal ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(mmuav_gazebo_gimbal ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
    mmuav_gazebo_ductedfan_motor_model mmuav_gazebo_gimbal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

