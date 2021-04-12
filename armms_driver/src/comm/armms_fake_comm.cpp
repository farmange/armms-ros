//============================================================================
// Name        : armms_fake_comm.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_driver/comm/armms_fake_comm.h"

using namespace KinovaApi;

namespace armms_driver
{
ArmmsFakeComm::ArmmsFakeComm()
{
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
  jointPositionHall = 200.0;
  jointPositionOptical = 200.0;
  return 0;
}

int ArmmsFakeComm::setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                      float& jointSpeed, float& jointTorque)
{
  jointPositionHall = 200.0;
  return 0;
}

int ArmmsFakeComm::initializeActuator(float& jointPositionOptical)
{
  jointPositionOptical = 200.0;
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
  jointPositionHall = 200.0;
  return 0;
}
}  // namespace armms_driver
