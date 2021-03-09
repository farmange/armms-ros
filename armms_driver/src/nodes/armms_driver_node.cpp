//============================================================================
// Name        : armms_driver_node.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : A ROS driver for controlling the ARMMS robotic components
//============================================================================
// TODO check all kinova and niryo word occurence
// TODO review all doc description
#include "armms_driver/armms_driver.h"

#include <ros/ros.h>

using namespace armms_driver;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_driver");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ArmmsDriver nd;

  ros::waitForShutdown();

  ROS_INFO("shutdown node");
}
