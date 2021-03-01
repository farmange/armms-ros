//============================================================================
// Name        : armms_switch_limit.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "std_msgs/Bool.h"

#include "armms_rpi/armms_switch_limit.h"

namespace armms_rpi
{
ArmmsSwitchLimit::ArmmsSwitchLimit(const ros::NodeHandle& nh) : nh_(nh)
{
  initializePublishers_();
}

void ArmmsSwitchLimit::initializePublishers_()
{
  ROS_DEBUG_NAMED("ArmmsSwitchLimit", "initializePublishers");
  switch_limit_pub_ = nh_.advertise<std_msgs::Bool>("/armms_rpi/limit_switch", 1);
}

void ArmmsSwitchLimit::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsSwitchLimit", "retrieveParameters");
  ros::param::get("~switch_limit_pin", switch_limit_pin_);
}

}  // namespace armms_rpi
