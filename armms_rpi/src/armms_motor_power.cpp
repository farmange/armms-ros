//============================================================================
// Name        : armms_motor_power.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_motor_power.h"

namespace armms_rpi
{
ArmmsMotorPower::ArmmsMotorPower(const ros::NodeHandle& nh) : nh_(nh)
{
}

void ArmmsMotorPower::initializePublishers_()
{
  ROS_DEBUG_NAMED("ArmmsMotorPower", "initializePublishers");
  power_button_pub_ = nh_.advertise<std_msgs::Bool>("/armms_rpi/power_button", 1);
}

void ArmmsSwitchLimit::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsSwitchLimit", "retrieveParameters");
  ros::param::get("~power_button_pin", power_button_pin_);
}
}  // namespace armms_rpi

using namespace armms_rpi;
