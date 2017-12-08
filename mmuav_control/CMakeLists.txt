cmake_minimum_required(VERSION 2.8.3)
project(mmuav_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    rospack
    std_msgs
    dynamic_reconfigure
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
    )
    
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(DualArmManipulatorDirectKinematics src/DualArmManipulatorDirectKinematics.cpp)
target_link_libraries(DualArmManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_library(DualArmManipulatorInverseKinematics src/DualArmManipulatorInverseKinematics.cpp)
target_link_libraries(DualArmManipulatorInverseKinematics DualArmManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_executable(dual_arm_manipulator_inverse_kinematics_node src/dual_arm_manipulator_inverse_kinematics_node.cpp)
target_link_libraries(dual_arm_manipulator_inverse_kinematics_node DualArmManipulatorDirectKinematics DualArmManipulatorInverseKinematics ${catkin_LIBRARIES} yaml-cpp)


#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})