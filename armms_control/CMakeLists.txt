cmake_minimum_required(VERSION 2.8.3)
project(armms_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  armms_msgs
)

# export library to other packages
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs
    armms_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ARMMS driver node
add_executable(armms_at1x_control
  src/armms_at1x_control.cpp
)
target_link_libraries(armms_at1x_control ${catkin_LIBRARIES})

add_dependencies(armms_at1x_control
  ${catkin_EXPORTED_TARGETS}
)
#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(TARGETS armms_at1x_control 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
