//============================================================================
// Name        : armms_power_button_led.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_POWER_BUTTON_LED_H
#define ARMMS_RPI_POWER_BUTTON_LED_H

#include <ros/ros.h>

namespace armms_rpi
{
class ArmmsPowerButtonLed
{
public:
  ArmmsPowerButtonLed(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
};

}  // namespace armms_rpi
#endif
