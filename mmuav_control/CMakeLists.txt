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
    config/UavGeometryControlParams.cfg
    )
    
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(MmuavParameters src/MmuavParameters.cpp)
target_link_libraries(MmuavParameters ${catkin_LIBRARIES})

add_library(NonlinearFilters src/NonlinearFilters.cpp)
target_link_libraries(NonlinearFilters ${catkin_LIBRARIES})

add_library(GeometricHelper src/GeometricHelper.cpp)
target_link_libraries(GeometricHelper ${catkin_LIBRARIES})

add_executable(UavGeometryControl src/UavGeometryControl.cpp)
target_link_libraries(UavGeometryControl ${catkin_LIBRARIES} 
    NonlinearFilters GeometricHelper MmuavParameters yaml-cpp)

#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
