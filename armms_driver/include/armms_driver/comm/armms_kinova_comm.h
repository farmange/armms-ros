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

namespace armms_driver
{
class ArmmsKinovaComm : public ArmmsBaseComm
{
public:
  ArmmsKinovaComm();

private:
  KinovaApi::APILayer* driver_;

  int loadLibrary_(const char* libname) = 0;
};

}  // namespace armms_driver
#endif  // ARMMS_DRIVER_ARMMS_KINOVA_COMM_H
