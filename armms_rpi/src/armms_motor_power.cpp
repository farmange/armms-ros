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
  retrieveParameters_();
  initializeServices_();
  motor_power_state_ = 0;

  pinMode(motor_power_pin_, OUTPUT);
}

ArmmsMotorPower::~ArmmsMotorPower()
{
  /* Stop motor power */
  digitalWrite(motor_power_pin_, 0);
}

void ArmmsMotorPower::update(bool& motor_power)
{
  /* Low side commutation and normally closed contact*/
  digitalWrite(motor_power_pin_, motor_power_state_);
  motor_power = motor_power_state_;
}

void ArmmsMotorPower::initializeServices_()
{
  set_motor_power_service_ =
      nh_.advertiseService("/armms_rpi/set_motor_power", &ArmmsMotorPower::callbackSetMotorPower_, this);
}

void ArmmsMotorPower::retrieveParameters_()
{
  ros::param::get("~motor_power_pin", motor_power_pin_);
}

bool ArmmsMotorPower::callbackSetMotorPower_(armms_msgs::SetMotorPower::Request& req,
                                             armms_msgs::SetMotorPower::Response& res)
{
  if (req.power_state == 1)
  {
    motor_power_state_ = 1;
  }
  else if (req.power_state == 0)
  {
    motor_power_state_ = 0;
  }
  else
  {
    ROS_WARN_NAMED("ArmmsMotorPower", "Wrong motor power state requested (must be 0 or 1).");
    return false;
  }
  return true;
}

}  // namespace armms_rpi