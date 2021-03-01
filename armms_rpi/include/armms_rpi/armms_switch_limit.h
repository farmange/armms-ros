//============================================================================
// Name        : armms_switch_limit.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_SWITCH_LIMIT_H
#define ARMMS_RPI_SWITCH_LIMIT_H

#include <ros/ros.h>

namespace armms_rpi
{
class ArmmsSwitchLimit
{
public:
  ArmmsSwitchLimit(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
  ros::Publisher switch_limit_pub_;

  int switch_limit_pin_;
  void retrieveParameters_();
  void initializePublishers_();
};

}  // namespace armms_rpi
#endif