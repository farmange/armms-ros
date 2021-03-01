//============================================================================
// Name        : armms_rpi.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_rpi.h"

namespace armms_rpi
{
ArmmsRpi::ArmmsRpi()
{
  ROS_INFO("Create power button led");
  power_button_led_.reset(new ArmmsPowerButtonLed(nh_));
  ROS_INFO("Create user button");
  user_button_.reset(new ArmmsUserButton(nh_));
  ROS_INFO("Create motor power");
  motor_power_.reset(new ArmmsMotorPower(nh_));
  ROS_INFO("Create switch limit");
  switch_limit_.reset(new ArmmsSwitchLimit(nh_));
}

}  // namespace armms_rpi

using namespace armms_rpi;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_rpi_node");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ArmmsRpi rpi;

  ros::waitForShutdown();

  ROS_INFO("shutdown node");
}