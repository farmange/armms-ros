//============================================================================
// Name        : armms_bag_recorder.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#include "armms_driver/armms_bag_recorder.h"

#include "std_msgs/Empty.h"
#include "armms_msgs/SetInt.h"

namespace armms_driver
{
ArmmsBagRecorder::ArmmsBagRecorder(const ros::NodeHandle& nh) : nh_(nh)
{
  initializeServices_();
}

void ArmmsBagRecorder::start()
{
  std_srvs::Empty msg;
  if (!start_bag_rec_srv_.call(msg))
  {
    ROS_ERROR_NAMED("ArmmsBagRecorder", "Cannot call \"start\" service");
  }
}

void ArmmsBagRecorder::stop()
{
  std_srvs::Empty msg;
  if (!stop_bag_rec_srv_.call(msg))
  {
    ROS_ERROR_NAMED("ArmmsBagRecorder", "Cannot call \"stop\" service");
  }
}

void ArmmsBagRecorder::initializeServices_()
{
  ros::service::waitForService("/armms_recorder/start");
  ros::service::waitForService("/armms_recorder/stop");
  start_bag_rec_srv_ = nh_.serviceClient<std_srvs::Empty>("/armms_recorder/start");
  stop_bag_rec_srv_ = nh_.serviceClient<std_srvs::Empty>("/armms_recorder/stop");
}

}  // namespace armms_driver