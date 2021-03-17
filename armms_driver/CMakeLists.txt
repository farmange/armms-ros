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

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

# export library to other packages
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES armms_hardware_interface
  CATKIN_DEPENDS roscpp rospy std_msgs armms_msgs controller_manager_msgs controller_manager
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ARMMS driver node
add_executable(armms_driver
src/nodes/armms_driver_node.cpp
src/armms_driver.cpp
src/armms_api.cpp
src/armms_hardware_interface.cpp
src/armms_bag_recorder.cpp
)
# target_link_libraries(armms_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES} armms_libkinova kinovadrv)
target_link_libraries(armms_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES} kinovadrv)


#############
## Install ##
#############

# Copy over .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/libkinovadrv.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(TARGETS armms_driver 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
