cmake_minimum_required(VERSION 2.8.3)
project(armms_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  RpiInterface.msg
  WebInterface.msg
)

add_service_files(
  FILES
  ButtonEvent.srv
  SetInt.srv
  SetLedColor.srv
  SetMotorPower.srv
  SetVelocitySetpoint.srv
)

# add_action_files(DIRECTORY action
#   FILES
# )

generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

