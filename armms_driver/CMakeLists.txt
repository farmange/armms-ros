cmake_minimum_required(VERSION 2.8.3)
project(armms_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  roscpp
  rospy
  std_msgs
  armms_msgs
  controller_manager_msgs
  controller_manager
)

# export library to other packages
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    armms_msgs 
    controller_manager_msgs 
    controller_manager
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ARMMS driver node
add_executable(${PROJECT_NAME}
src/nodes/armms_driver_node.cpp
src/armms_driver.cpp
src/comm/armms_fake_comm.cpp
src/comm/armms_kinova_comm.cpp
src/armms_hardware_interface.cpp
src/armms_bag_recorder.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} -lkinova)

add_dependencies(${PROJECT_NAME}
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
install(TARGETS ${PROJECT_NAME} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
