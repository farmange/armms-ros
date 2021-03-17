//============================================================================
// Name        : armms_rpi.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include <ros/callback_queue.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <fstream>

#include "armms_rpi/armms_rpi.h"
#include "armms_msgs/RpiInterface.h"

namespace armms_rpi
{
ArmmsRpi::ArmmsRpi()
{
  rpi_loop_rate_ = 0;
  retrieveParameters_();
  initializePublishers_();

  if (rpi_loop_rate_ <= 0)
  {
    ROS_ERROR("Bad sampling frequency value (%d) for RPI thread", rpi_loop_rate_);
    return;
  }

  if (wiringPiSetup() == -1)
  {
    ROS_ERROR("wiringPiSetup error !");
    return;
  }

  ROS_INFO("Create power button led");
  power_button_led_.reset(new ArmmsPowerButtonLed(nh_));
  ROS_INFO("Create user button");
  user_button_.reset(new ArmmsUserButton(nh_));
  ROS_INFO("Create motor power");
  motor_power_.reset(new ArmmsMotorPower(nh_));
  ROS_INFO("Create switch limit");
  switch_limit_.reset(new ArmmsSwitchLimit(nh_));
  ROS_INFO("Create shutdown manager");
  shutdown_manager_.reset(new ArmmsShutdownManager(nh_, power_button_led_.get()));
  ROS_INFO("Create diagnostic thread");
  rpi_diagnostics_.reset(new ArmmsDiagnostics(nh_));

  ros::Duration rpi_update_time = ros::Duration(1.0 / rpi_loop_rate_);
  rpi_non_rt_loop_ = nh_.createTimer(rpi_update_time, &ArmmsRpi::update_, this);

  ros::spin();
}

void ArmmsRpi::retrieveParameters_()
{
  ros::param::get("~rpi_loop_rate", rpi_loop_rate_);
}

void ArmmsRpi::initializePublishers_()
{
  rpi_interface_pub_ = nh_.advertise<armms_msgs::RpiInterface>("/armms_rpi/rpi_interface", 1);
}

void ArmmsRpi::update_(const ros::TimerEvent&)
{
  bool motor_power;
  bool switch_limit;
  bool user_button_up;
  bool user_button_down;

  power_button_led_->update();
  user_button_->update(user_button_up, user_button_down);
  motor_power_->update(motor_power);
  switch_limit_->update(switch_limit);

  armms_msgs::RpiInterface msg;
  msg.motor_power = motor_power;
  msg.switch_limit = switch_limit;
  msg.user_button_up = user_button_up;
  msg.user_button_down = user_button_down;
  msg.rpi_temperature = rpi_diagnostics_->getCpuTemperature();
  rpi_interface_pub_.publish(msg);
}
}  // namespace armms_rpi

using namespace armms_rpi;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_rpi_node");

  ros::NodeHandle nh;

  ArmmsRpi rpi;

  ROS_INFO("shutdown node");
}