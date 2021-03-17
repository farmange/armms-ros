//============================================================================
// Name        : armms_shutdown_manager.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_shutdown_manager.h"

namespace armms_rpi
{
ArmmsShutdownManager::ArmmsShutdownManager(const ros::NodeHandle& nh, ArmmsPowerButtonLed* power_button_led)
  : nh_(nh), power_button_led_(power_button_led)
{
  initializeServices_();
}

void ArmmsShutdownManager::shutdown()
{
  system_thread_.reset(new std::thread(boost::bind(&ArmmsShutdownManager::shutdownThread_, this)));
}

void ArmmsShutdownManager::reboot()
{
  system_thread_.reset(new std::thread(boost::bind(&ArmmsShutdownManager::rebootThread_, this)));
}

void ArmmsShutdownManager::shutdownThread_()
{
  ROS_INFO("Execute shutdown thread...");
  power_button_led_->setLedColor(255, 0, 0, 0);
  /* The following command requires that execution access was grant to the user
   * (with systemd only primary group is defined at startup).
   */
  std::system("sudo /sbin/shutdown -h now");
}

void ArmmsShutdownManager::rebootThread_()
{
  ROS_INFO("Execute reboot thread...");
  power_button_led_->setLedColor(255, 0, 0, 0);
  /* The following command requires that execution access was grant to the user
   * (with systemd only primary group is defined at startup).
   */
  std::system("sudo reboot");
}

void ArmmsShutdownManager::initializeServices_()
{
  shutdown_service_ = nh_.advertiseService("/armms_rpi/shutdown_rpi", &ArmmsShutdownManager::callbackShutdown_, this);
}

bool ArmmsShutdownManager::callbackShutdown_(armms_msgs::SetInt::Request& req, armms_msgs::SetInt::Response& res)
{
  if (req.value == 1)
  {
    shutdown();
  }
  else if (req.value == 2)
  {
    reboot();
  }
  else
  {
    return false;
  }
  return true;
}

}  // namespace armms_rpi

using namespace armms_rpi;
