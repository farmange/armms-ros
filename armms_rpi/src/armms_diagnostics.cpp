//============================================================================
// Name        : armms_diagnostics.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#include "armms_rpi/armms_diagnostics.h"

namespace armms_rpi
{
ArmmsDiagnostics::ArmmsDiagnostics(const ros::NodeHandle& nh) : nh_(nh)
{
  cpu_temperature_ = 0;
  loop_rate_ = 0;
  retrieveParameters_();
  initializeServices_();
  if (loop_rate_ <= 0)
  {
    ROS_ERROR("Bad sampling frequency value (%d)", loop_rate_);
    return;
  }

  ros::Duration update_time = ros::Duration(1.0 / loop_rate_);
  diag_non_rt_loop_ = nh_.createTimer(update_time, &ArmmsDiagnostics::update_, this);

  // ros::spin();
}

float ArmmsDiagnostics::getCpuTemperature()
{
  return cpu_temperature_;
}

void ArmmsDiagnostics::retrieveParameters_()
{
  ros::param::get("~diag_loop_rate", loop_rate_);
}

void ArmmsDiagnostics::initializeServices_()
{
  // ros::service::waitForService("/armms_rpi/shutdown_rpi");
  shutdown_srv_ = nh_.serviceClient<armms_msgs::SetInt>("/armms_rpi/shutdown_rpi");
}

void ArmmsDiagnostics::readCpuTemperature_()
{
  std::fstream temp_fstream("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);

  float cpu_temp;
  temp_fstream >> cpu_temp;
  if (cpu_temp > 0)
  {
    cpu_temperature_ = cpu_temp / 1000.0;
  }
}

void ArmmsDiagnostics::update_(const ros::TimerEvent&)
{
  readCpuTemperature_();
  ROS_DEBUG("Read CPU temperature : %f", cpu_temperature_);

  // check if Rpi is too hot
  if (cpu_temperature_ > 75.0)
  {
    ROS_ERROR("Rpi temperature is really high !");
  }
  if (cpu_temperature_ > 85.0)
  {
    ROS_ERROR("Rpi is too hot, shutdown to avoid any damage");
    armms_msgs::SetInt msgShutdown;
    msgShutdown.request.value = 1;  // shutdown
    if (!shutdown_srv_.call(msgShutdown))
    {
      ROS_ERROR("Problem when calling shutdown service");
    }
  }
}
}  // namespace armms_rpi
