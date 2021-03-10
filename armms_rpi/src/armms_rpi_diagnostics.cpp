//============================================================================
// Name        : armms_rpi_diagnostics.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#include "armms_rpi/armms_rpi_diagnostics.h"

namespace armms_rpi
{
ArmmsRpiDiagnostics::ArmmsRpiDiagnostics()
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
  non_realtime_loop_ = nh_.createTimer(update_time, &ArmmsRpiDiagnostics::update_, this);

  ros::spin();
}

void ArmmsRpiDiagnostics::retrieveParameters_()
{
  ros::param::get("~diag_loop_rate", loop_rate_);
}

void ArmmsRpiDiagnostics::initializeServices_()
{
  ros::service::waitForService("/armms_rpi/shutdown_rpi");
  shutdown_srv_ = nh_.serviceClient<armms_msgs::SetInt>("/armms_rpi/shutdown_rpi");
}

void ArmmsRpiDiagnostics::readCpuTemperature_()
{
  std::fstream temp_fstream("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);

  int cpu_temp;
  temp_fstream >> cpu_temp;
  if (cpu_temp > 0)
  {
    cpu_temperature_ = cpu_temp / 1000;
  }
}

void ArmmsRpiDiagnostics::update_(const ros::TimerEvent&)
{
  readCpuTemperature_();
  ROS_INFO("CPU temperature : %d", cpu_temperature_);

  // check if Rpi is too hot
  if (cpu_temperature_ > 75)
  {
    ROS_ERROR("Rpi temperature is really high !");
  }
  if (cpu_temperature_ > 85)
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

using namespace armms_rpi;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_rpi_diag_node");

  ros::NodeHandle nh;

  ArmmsRpiDiagnostics rpi_diag;

  ROS_INFO("shutdown node");
}