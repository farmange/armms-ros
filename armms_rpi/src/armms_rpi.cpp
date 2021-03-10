//============================================================================
// Name        : armms_rpi.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_rpi.h"
#include <signal.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
// // Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
namespace armms_rpi
{
ArmmsRpi::ArmmsRpi()
{
  loop_rate_ = 0;
  retrieveParameters_();

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
  shutdown_manager_.reset(new ArmmsShutdownManager(nh_));

  if (loop_rate_ <= 0)
  {
    ROS_ERROR("Bad sampling frequency value (%d)", loop_rate_);
    return;
  }

  ros::Duration update_time = ros::Duration(1.0 / loop_rate_);
  non_realtime_loop_ = nh_.createTimer(update_time, &ArmmsRpi::update_, this);

  ros::spin();
}

void ArmmsRpi::retrieveParameters_()
{
  ros::param::get("~rpi_loop_rate", loop_rate_);
}

void ArmmsRpi::update_(const ros::TimerEvent&)
{
  power_button_led_->update();
  user_button_->update();
  motor_power_->update();
  switch_limit_->update();
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