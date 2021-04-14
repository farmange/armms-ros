//============================================================================
// Name        : armms_kinova_comm.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_DRIVER_ARMMS_KINOVA_COMM_H
#define ARMMS_DRIVER_ARMMS_KINOVA_COMM_H

#include <dlfcn.h>
#include <ros/ros.h>
#include <armms_driver/comm/armms_base_comm.h>
#include "kinovadrv/kinovadrv.h"

namespace armms_driver
{
class ArmmsKinovaComm : public ArmmsBaseComm
{
public:
  ArmmsKinovaComm();
  int init(const std::string device, const bool& debug_log = false);
  int initializeActuator(float& jointPositionOptical);
  // TODO could be remove
  int clearError();
  int startMotorControl();
  int stopMotorControl();

  int setPositionCommandExt(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                    float& jointSpeed, float& jointTorque, float& jointPMW, float& jointPositionOptical,
                                    short& jointAccelX, short& jointAccelY, short& jointAccelZ, short& jointTemp);
  int setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                 float& jointSpeed, float& jointTorque);
  int getActualPosition(float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                                float& jointTorque);
private:
  KinovaApi::APILayer* driver_;

  int loadLibrary_(const char* libname);
};

}  // namespace armms_driver
#endif  // ARMMS_DRIVER_ARMMS_KINOVA_COMM_H
