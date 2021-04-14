//============================================================================
// Name        : armms_fake_comm.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_driver/comm/armms_fake_comm.h"
#include <math.h>

namespace armms_driver
{
ArmmsFakeComm::ArmmsFakeComm()
{
  omega_t_ = 0.0;
  torque_ = -1.2;
  position_ = 200.0;
}

int ArmmsFakeComm::init(const std::string device, const bool& debug_log)
{
  ROS_INFO("Loading fake communication...");
  return 0;
}

int ArmmsFakeComm::setPositionCommandExt(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                         float& jointSpeed, float& jointTorque, float& jointPMW,
                                         float& jointPositionOptical, short& jointAccelX, short& jointAccelY,
                                         short& jointAccelZ, short& jointTemp)
{
  jointPositionHall = jointCommand;
  torque_ = -1.2  + (2.* sin (omega_t_));
  omega_t_ = omega_t_ + 0.01;
  if(omega_t_ > (3.14*2))
  {
    omega_t_ = 0;
  }
  jointTorque = torque_;
  position_ = jointPositionHall;
  return 0;
}

int ArmmsFakeComm::setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                      float& jointSpeed, float& jointTorque)
{
  jointPositionHall = jointCommand;
  torque_ = -1.2  + (2.* sin (omega_t_));
  omega_t_ = omega_t_ + 0.01;
  if(omega_t_ > (3.14*2))
  {
    omega_t_ = 0;
  }
  jointTorque = torque_;
  position_ = jointPositionHall;
  return 0;
}

int ArmmsFakeComm::initializeActuator(float& jointPositionOptical)
{
  jointPositionOptical = position_;
  return 0;
}

int ArmmsFakeComm::clearError()
{
  return 0;
}

int ArmmsFakeComm::startMotorControl()
{
  return 0;
}

int ArmmsFakeComm::stopMotorControl()
{
  return 0;
}

// TODO documentation
int ArmmsFakeComm::getActualPosition(float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                                     float& jointTorque)
{
  jointTorque = torque_;
  jointPositionHall = position_;
  return 0;
}
}  // namespace armms_driver
