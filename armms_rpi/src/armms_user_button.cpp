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

  pinMode(btn_up_pin_, INPUT);
  pinMode(btn_down_pin_, INPUT);
  pullUpDnControl(btn_up_pin_, PUD_OFF);
  pullUpDnControl(btn_down_pin_, PUD_OFF);
}

void ArmmsUserButton::update(bool& button_up, bool& button_down)
{
  /* Low side commutation */
  button_up = 1 - digitalRead(btn_up_pin_);
  button_down = 1 - digitalRead(btn_down_pin_);
}

void ArmmsUserButton::retrieveParameters_()
{
  ros::param::get("~user_btn_up_pin", btn_up_pin_);
  ros::param::get("~user_btn_down_pin", btn_down_pin_);
}

}  // namespace armms_rpi