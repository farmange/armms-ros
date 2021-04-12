//============================================================================
// Name        : armms_base_comm.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_DRIVER_ARMMS_BASE_COMM_H
#define ARMMS_DRIVER_ARMMS_BASE_COMM_H

#include <ros/ros.h>

namespace armms_driver
{
class ArmmsBaseComm
{
public:
  ArmmsBaseComm();
  int init(const std::string device, const bool& debug_log = false) = 0;

  int initializeActuator(float& jointPositionOptical) = 0;

  int clearError() = 0;
  int startMotorControl() = 0;
  int stopMotorControl() = 0;

  int setPositionCommandExt(const float& jointCommand, float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                            float& jointTorque, float& jointPMW, float& jointPositionOptical, short& jointAccelX,
                            short& jointAccelY, short& jointAccelZ, short& jointTemp) = 0;
  int setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                         float& jointTorque) = 0;
  int getActualPosition(float& jointCurrent, float& jointPositionHall, float& jointSpeed, float& jointTorque) = 0;

private:
  ros::NodeHandle nh_;
  uint16_t jointAddress_;
};

}  // namespace armms_driver
#endif  // ARMMS_DRIVER_ARMMS_BASE_COMM_H
