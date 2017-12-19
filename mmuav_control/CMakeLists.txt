cmake_minimum_required(VERSION 2.8.3)
project(mmuav_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    rospack
    std_msgs
    dynamic_reconfigure
    mmuav_msgs
)

add_definitions(-std=c++11)

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
    config/UavPositionCtlParams.cfg
    )
    
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack mmuav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

add_library(DualArmManipulatorDirectKinematics src/DualArmManipulatorDirectKinematics.cpp)
target_link_libraries(DualArmManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_library(DualArmManipulatorInverseKinematics src/DualArmManipulatorInverseKinematics.cpp)
target_link_libraries(DualArmManipulatorInverseKinematics DualArmManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_executable(dual_arm_manipulator_control_node src/dual_arm_manipulator_control.cpp)
target_link_libraries(dual_arm_manipulator_control_node DualArmManipulatorDirectKinematics DualArmManipulatorInverseKinematics ${catkin_LIBRARIES} yaml-cpp)

add_executable(uav_attitude_control src/uav_attitude_control.cpp)
target_link_libraries(uav_attitude_control pid ${catkin_LIBRARIES})

add_executable(uav_position_control src/uav_position_control.cpp)
target_link_libraries(uav_position_control pid ${catkin_LIBRARIES})

add_executable(uav_force_control src/uav_force_control.cpp)
target_link_libraries(uav_force_control pid ${catkin_LIBRARIES})


#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})