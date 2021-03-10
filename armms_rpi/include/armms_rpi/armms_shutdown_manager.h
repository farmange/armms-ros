//============================================================================
// Name        : armms_shutdown_manager.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : TODO all doc of armms_rpi
//============================================================================

#ifndef ARMMS_RPI_SHUTDOWN_MANAGER_H
#define ARMMS_RPI_SHUTDOWN_MANAGER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <thread>

#include "armms_msgs/SetInt.h"
#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/ButtonEvent.h"

namespace armms_rpi
{
class ArmmsShutdownManager
{
public:
  ArmmsShutdownManager(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer shutdown_service_;
  boost::shared_ptr<std::thread> shutdown_thread_;

  void initializeServices_();
  void shutdownThread_();
  void rebootThread_();
  bool callbackShutdown_(armms_msgs::SetInt::Request& req, armms_msgs::SetInt::Response& res);
};

}  // namespace armms_rpi
#endif
