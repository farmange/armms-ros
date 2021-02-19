//============================================================================
// Name        : armms_api.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_DRIVER_ARMMS_API_H
#define ARMMS_DRIVER_ARMMS_API_H

#include <dlfcn.h>
#include <ros/ros.h>

// #include <iostream>

#include "kinovadrv/kinovadrv.h"
// #include "kinova/Kinova.API.EthCommandLayerUbuntu.h"
// #include "kinova/KinovaTypes.h"

namespace armms
{
class ArmmsAPI
{
public:
  ArmmsAPI(ros::NodeHandle nh);
  int init(const std::string device, const bool& debug_log = false);
  // int getPosition();

  //   int initializeKinovaAPIFunctions(KinovaAPIType connection_type);
  int loadLibrary(const char* kinova_comm_lib);
  int initializeActuator(float& jointPositionOptical);
  int startMotorControl();
  int stopMotorControl();
  
  int setPositionCommandExt(const float& jointCommand, float& jointPositionOptical, float& jointTorque);
  int setPositionCommand(const float& jointCommand, float& jointPositionOptical, float& jointTorque);
  int getJointStatesSetCommand(const float& jointCommand, float& jointPositionOptical, float& jointSpeed,
                               float& jointTorque);

private:
  ros::NodeHandle nh_;
  KinovaApi::APILayer* driver_;
  uint16_t jointAddress_;
};

}  // namespace armms
#endif  // ARMMS_DRIVER_ARMMS_API_H
