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
  retrieveParameters_();

  pinMode(switch_limit_pin_, INPUT);
  pullUpDnControl(switch_limit_pin_, PUD_OFF);
}

void ArmmsSwitchLimit::update(bool& switch_limit)
{
  /* Low side commutation and normally closed contact*/
  switch_limit = digitalRead(switch_limit_pin_);
}

void ArmmsSwitchLimit::retrieveParameters_()
{
  ros::param::get("~switch_limit_pin", switch_limit_pin_);
}

}  // namespace armms_rpi
