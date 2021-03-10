//============================================================================
// Name        : armms_api_diagnostics.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_ARMMS_RPI_DIAGNOSTICS_H
#define ARMMS_RPI_ARMMS_RPI_DIAGNOSTICS_H

#include <ros/ros.h>
#include <fstream>

#include "armms_msgs/SetInt.h"

namespace armms_rpi
{
class ArmmsRpiDiagnostics
{
public:
  ArmmsRpiDiagnostics();

  int getCpuTemperature();

  void startReadingData();

private:
  ros::NodeHandle nh_;

  ros::Timer non_realtime_loop_;
  ros::ServiceClient shutdown_srv_;
  int cpu_temperature_;
  int loop_rate_;
  void retrieveParameters_();
  void initializeServices_();
  void readCpuTemperature_();
  void update_(const ros::TimerEvent&);
};

}  // namespace armms_rpi
#endif
