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
  initializePublishers_();
  switch_limit_state_ = 0;

  pinMode(switch_limit_pin_, INPUT);
  pullUpDnControl(switch_limit_pin_, PUD_OFF);
}

void ArmmsSwitchLimit::update()
{
  /* Low side commutation and normally closed contact*/
  switch_limit_state_ = digitalRead(switch_limit_pin_);
  std_msgs::Bool msg;
  if (switch_limit_state_ != 0)
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  switch_limit_pub_.publish(msg);
}

void ArmmsSwitchLimit::initializePublishers_()
{
  ROS_DEBUG_NAMED("ArmmsSwitchLimit", "initializePublishers");
  switch_limit_pub_ = nh_.advertise<std_msgs::Bool>("/armms_rpi/switch_limit", 1);
}

void ArmmsSwitchLimit::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsSwitchLimit", "retrieveParameters");
  ros::param::get("~switch_limit_pin", switch_limit_pin_);
}

}  // namespace armms_rpi
