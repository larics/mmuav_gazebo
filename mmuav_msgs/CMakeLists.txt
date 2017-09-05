cmake_minimum_required(VERSION 2.8.3)
project(mmuav_msgs)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
  FILES
  MotorSpeed.msg
  PIDController.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)