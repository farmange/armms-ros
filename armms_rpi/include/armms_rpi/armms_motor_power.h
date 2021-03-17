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
#include <wiringPi.h>
#include "std_msgs/Bool.h"
#include "armms_msgs/SetMotorPower.h"

namespace armms_rpi
{
class ArmmsMotorPower
{
public:
  ArmmsMotorPower(const ros::NodeHandle& nh);
  ~ArmmsMotorPower();
  void update(bool& motor_power);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer set_motor_power_service_;
  int motor_power_pin_;
  int motor_power_state_;

  void retrieveParameters_();
  void initializeServices_();
  bool callbackSetMotorPower_(armms_msgs::SetMotorPower::Request& req, armms_msgs::SetMotorPower::Response& res);
};
}  // namespace armms_rpi
#endif
