//============================================================================
// Name        : armms_motor_power.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_MOTOR_POWER_H
#define ARMMS_RPI_MOTOR_POWER_H

#include <ros/ros.h>

namespace armms_rpi
{
class ArmmsMotorPower
{
public:
  ArmmsMotorPower(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
};

}  // namespace armms_rpi
#endif
