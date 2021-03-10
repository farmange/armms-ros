//============================================================================
// Name        : armms_user_button.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_user_button.h"

namespace armms_rpi
{
ArmmsUserButton::ArmmsUserButton(const ros::NodeHandle& nh) : nh_(nh)
{
  retrieveParameters_();
  initializePublishers_();
  btn_up_state_ = 0;
  btn_down_state_ = 0;

  pinMode(btn_up_pin_, INPUT);
  pinMode(btn_down_pin_, INPUT);
  pullUpDnControl(btn_up_pin_, PUD_OFF);
  pullUpDnControl(btn_down_pin_, PUD_OFF);
}

void ArmmsUserButton::update()
{
  /* Low side commutation */
  btn_up_state_ = 1 - digitalRead(btn_up_pin_);
  btn_down_state_ = 1 - digitalRead(btn_down_pin_);

  std_msgs::Bool msg;

  if (btn_up_state_ != 0)
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  btn_up_pub_.publish(msg);

  if (btn_down_state_ != 0)
  {
    msg.data = true;
  }
  else
  {
    msg.data = false;
  }
  btn_down_pub_.publish(msg);
}

void ArmmsUserButton::initializePublishers_()
{
  btn_up_pub_ = nh_.advertise<std_msgs::Bool>("/armms_rpi/user_button_up", 1);
  btn_down_pub_ = nh_.advertise<std_msgs::Bool>("/armms_rpi/user_button_down", 1);
}

void ArmmsUserButton::retrieveParameters_()
{
  ros::param::get("~user_btn_up_pin", btn_up_pin_);
  ros::param::get("~user_btn_down_pin", btn_down_pin_);
}

}  // namespace armms_rpi