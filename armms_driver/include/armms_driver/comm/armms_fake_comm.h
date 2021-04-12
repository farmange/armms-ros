//============================================================================
// Name        : armms_fake_comm.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_DRIVER_ARMMS_FAKE_COMM_H
#define ARMMS_DRIVER_ARMMS_FAKE_COMM_H

#include <ros/ros.h>

namespace armms_driver
{
class ArmmsFakeComm : public ArmmsBaseComm
{
public:
  ArmmsFakeComm();
};

}  // namespace armms_driver
#endif  // ARMMS_DRIVER_ARMMS_FAKE_COMM_H
