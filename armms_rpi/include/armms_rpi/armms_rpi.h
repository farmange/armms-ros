//============================================================================
// Name        : armms_api.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_ARMMS_RPI_H
#define ARMMS_RPI_ARMMS_RPI_H

#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "armms_rpi/armms_switch_limit.h"
#include "armms_rpi/armms_power_button_led.h"
#include "armms_rpi/armms_diagnostics.h"
#include "armms_rpi/armms_user_button.h"
#include "armms_rpi/armms_motor_power.h"
#include "armms_rpi/armms_shutdown_manager.h"

namespace armms_rpi
{
class ArmmsRpi
{
public:
  ArmmsRpi();

private:
  ros::NodeHandle nh_;
  ros::Publisher rpi_interface_pub_;
  boost::shared_ptr<ArmmsPowerButtonLed> power_button_led_;
  boost::shared_ptr<ArmmsUserButton> user_button_;
  boost::shared_ptr<ArmmsMotorPower> motor_power_;
  boost::shared_ptr<ArmmsSwitchLimit> switch_limit_;
  boost::shared_ptr<ArmmsShutdownManager> shutdown_manager_;
  boost::shared_ptr<ArmmsDiagnostics> rpi_diagnostics_;

  int rpi_loop_rate_;
  ros::Timer rpi_non_rt_loop_;

  void retrieveParameters_();
  void initializePublishers_();
  void update_(const ros::TimerEvent&);
};

}  // namespace armms_rpi
#endif
