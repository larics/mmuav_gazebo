cmake_minimum_required(VERSION 2.8.3)
project(mmuav_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
    roslib
	mmuav_msgs
	mav_msgs
)

find_package(cmake_modules REQUIRED)

generate_dynamic_reconfigure_options(
    # Generic UAV parameters
    config/UavZCtlParams.cfg
    config/UavAttitudeCtlParams.cfg
    # MMUAV parameters
    config/MmuavZCtlParams.cfg
    config/MmuavAttitudeCtlParams.cfg
    # MMCUAV parameters
    config/MmcuavZCtlParams.cfg
    config/MmcuavAttitudeCtlParams.cfg
    # VPC MMCUAV parameters
    config/VpcMmcuavZCtlParams.cfg
    config/VpcMmcuavPositionCtlParams.cfg
    config/VpcMmcuavAttitudeCtlParams.cfg
    config/PositionCtlParams.cfg
    # Geometry control parameters
    config/UavGeometryControlParams.cfg
    # Joint control parameters
    config/JointCtlParams.cfg
    )
    
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# PID controller library
add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

# General MmuavParameters library
add_library(MmuavParameters src/MmuavParameters.cpp)
target_link_libraries(MmuavParameters ${catkin_LIBRARIES})

# Library for nonlinear filters
add_library(NonlinearFilters src/NonlinearFilters.cpp)
target_link_libraries(NonlinearFilters ${catkin_LIBRARIES})

# Helper library for geometric control
add_library(GeometricHelper src/GeometricHelper.cpp)
target_link_libraries(GeometricHelper ${catkin_LIBRARIES})

add_executable(UavGeometryControl src/UavGeometryControl.cpp)
target_link_libraries(UavGeometryControl ${catkin_LIBRARIES} 
    NonlinearFilters GeometricHelper MmuavParameters yaml-cpp)

add_executable(mmuav_joint_position_control src/mmuav_joint_position_control.cpp)
target_link_libraries(mmuav_joint_position_control pid ${catkin_LIBRARIES} yaml-cpp)

#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
