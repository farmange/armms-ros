//============================================================================
// Name        : armms_user_button.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_USER_BUTTON_H
#define ARMMS_RPI_USER_BUTTON_H

#include <ros/ros.h>

namespace armms_rpi
{
class ArmmsUserButton
{
public:
  ArmmsUserButton(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
};

}  // namespace armms_rpi
#endif
