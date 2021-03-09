//============================================================================
// Name        : armms_shutdown_manager.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_shutdown_manager.h"

namespace armms_rpi
{
ArmmsShutdownManager::ArmmsShutdownManager(const ros::NodeHandle& nh) : nh_(nh)
{
  retrieveParameters_();
  initializeServices_();
  ROS_INFO("Starting ros control thread...");
}

void ArmmsShutdownManager::shutdownThread_()
{
  ROS_DEBUG_NAMED("ArmmsShutdownManager", "shutdownThread_");
  std::system("killall -9 rosmaster");
  /* The following command requires that execution access was grant to the user
   * (with systemd only primary group is defined at startup).
   */
  std::system("sudo /sbin/shutdown -h now");
}

void ArmmsShutdownManager::rebootThread_()
{
  ROS_DEBUG_NAMED("ArmmsShutdownManager", "rebootThread_");
  std::system("killall -9 rosmaster");
  /* The following command requires that execution access was grant to the user
   * (with systemd only primary group is defined at startup).
   */
  std::system("sudo reboot");
}

void ArmmsShutdownManager::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsShutdownManager", "retrieveParameters");
  // TODO remove unused function
}

void ArmmsShutdownManager::initializeServices_()
{
  ROS_DEBUG_NAMED("ArmmsShutdownManager", "initializeServices");
  shutdown_service_ = nh_.advertiseService("/armms_rpi/shutdown_rpi", &ArmmsShutdownManager::callbackShutdown_, this);
}

bool ArmmsShutdownManager::callbackShutdown_(armms_msgs::SetInt::Request& req, armms_msgs::SetInt::Response& res)
{
  ROS_DEBUG_NAMED("ArmmsShutdownManager", "callbackShutdown_");
  if (req.value == 1)
  {
    // send_shutdown_command_thread =
    //     threading.Timer(1.0, send_shutdown_command) send_shutdown_command_thread.start() return
    // {
    //   'status' : 200, 'message' : 'Robot is shutting down'
    // }
    shutdown_thread_.reset(new std::thread(boost::bind(&ArmmsShutdownManager::shutdownThread_, this)));
  }
  else if (req.value == 2)
  {
    // send_reboot_command_thread = threading.Timer(1.0, send_reboot_command) send_reboot_command_thread.start() return
    // {
    //   'status' : 200, 'message' : 'Robot is rebooting'
    // }
    shutdown_thread_.reset(new std::thread(boost::bind(&ArmmsShutdownManager::rebootThread_, this)));
  }
  else
  {
    // return
    // {
    //   'status' : 400, 'message' : 'Incorrect value: 1 for shutdown, 2 for reboot'
    // }
  }
  return true;
}

}  // namespace armms_rpi

using namespace armms_rpi;
