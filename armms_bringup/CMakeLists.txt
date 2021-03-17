cmake_minimum_required(VERSION 2.8.3)
project(armms_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
  launch/at1x_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
