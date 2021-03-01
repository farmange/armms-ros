cmake_minimum_required(VERSION 2.8.3)
project(armms_rpi)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  armms_msgs
)

# export library to other packages
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
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
add_executable(${PROJECT_NAME}
  src/armms_rpi.cpp
  src/armms_switch_limit.cpp
  src/armms_power_button_led.cpp
  src/armms_user_button.cpp
  src/armms_motor_power.cpp
)

#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "aarch64")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} 
    )
endif()

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
install(TARGETS armms_rpi 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
