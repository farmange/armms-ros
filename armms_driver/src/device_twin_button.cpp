/*
 *  device_freejoy.cpp
 *  Copyright (C) 2020 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"

#include "armms_driver/device_twin_button.h"
#include <wiringPi.h>
#include <softPwm.h>  // pthread ?

// TODO rename node in RPI IO
namespace input_device
{
DeviceTwinButton::DeviceTwinButton()
{
  retrieveParameters_();
  initializePublishers_();
  initializeSubscribers_();
  initializeServices_();

  /* Initialize buttons debounce timers */
  debounce_up_btn_ = ros::Time::now();
  debounce_down_btn_ = ros::Time::now();
  debounce_power_btn_ = ros::Time::now();

  /* Initialize input and output state */
  up_btn_state_ = 0;
  down_btn_state_ = 0;
  power_btn_state_ = 0;
  limit_switch_state_ = 0;
  red_led_state_ = 0;
  green_led_state_ = 0;
  blue_led_state_ = 0;
  supply_en_state_ = 0;
  led_blink_speed_ = 0;
  led_blink_counter_ = 0;
  led_blink_state_ = true;
  debounce_button_time_ = 0.2;
  button_action_ = BTN_NONE;

  if (wiringPiSetup() == -1)
  {
    ROS_ERROR_NAMED("DeviceTwinButton", "wiringPiSetup error !");
    exit(0);
  }
  // TODO try PWM ??? http://wiringpi.com/reference/software-pwm-library/
  pinMode(up_pin_, INPUT);
  pinMode(down_pin_, INPUT);
  pinMode(power_pin_, INPUT);
  pinMode(limit_switch_pin_, INPUT);
  pinMode(red_led_pin_, OUTPUT);
  pinMode(green_led_pin_, OUTPUT);
  pinMode(blue_led_pin_, OUTPUT);
  pinMode(supply_en_pin_, OUTPUT);

  pullUpDnControl(up_pin_, PUD_OFF);
  pullUpDnControl(down_pin_, PUD_OFF);
  pullUpDnControl(power_pin_, PUD_OFF);
  pullUpDnControl(limit_switch_pin_, PUD_OFF);

  if (softPwmCreate(red_led_pin_, 0, 100) != 0)
  {
    ROS_ERROR_NAMED("DeviceTwinButton", "red_led_pin_ softPwmCreate error !");
    exit(0);
  }

  if (softPwmCreate(green_led_pin_, 0, 100) != 0)
  {
    ROS_ERROR_NAMED("DeviceTwinButton", "green_led_pin_ softPwmCreate error !");
    exit(0);
  }

  if (softPwmCreate(blue_led_pin_, 0, 100) != 0)
  {
    ROS_ERROR_NAMED("DeviceTwinButton", "blue_led_pin_ softPwmCreate error !");
    exit(0);
  }

  // ros::Rate loop_rate = ros::Rate(sampling_frequency_);

  ros::Duration update_freq = ros::Duration(1.0 / sampling_frequency_);
  non_realtime_loop_ = n_.createTimer(update_freq, &DeviceTwinButton::update_, this);
  // ros::spin();
  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  ros::waitForShutdown();
}

void DeviceTwinButton::update_(const ros::TimerEvent& e)
{
  processInputs_();
  processOutputs_();
  std_msgs::Float64 joint_vel;
  if (up_btn_state_ == 1 && down_btn_state_ == 0)
  {
    joint_vel.data = +joint_max_spd_;
  }
  else if (up_btn_state_ == 0 && down_btn_state_ == 1)
  {
    joint_vel.data = -joint_max_spd_;
  }
  else
  {
    joint_vel.data = 0;
  }
  cmd_pub_.publish(joint_vel);
  // start_srv
  if (power_btn_state_ == 1)
  {
    if (power_btn_prev_state_ == 0)
    {
      /* Rising edge of button */
      press_time_ = ros::Time::now();
    }
    /* Raise event if button is maintained during long time */
    if (ros::Duration(ros::Time::now() - press_time_) > long_press_duration_)
    {
      /* Long press detected */
      button_action_ = BTN_LONG_PRESS;
    }
  }
  else
  {
    /* Falling edge detected */
    if (power_btn_prev_state_ == 1)
    {
      /* short press detected */
      button_action_ = BTN_SHORT_PRESS;
    }
  }
  /* Only send service request if an event is detected */
  if ((button_action_ == BTN_LONG_PRESS) || (button_action_ == BTN_SHORT_PRESS))
  {
    armms_msgs::ButtonEvent msgButton;
    msgButton.request.button_event = button_action_;

    if (!button_event_srv_.call(msgButton))
    {
      ROS_ERROR_NAMED("HLController", "Problem when raising button event");
      /* TODO handle error here */
    }
  }
  button_action_ = BTN_NONE;
  power_btn_prev_state_ = power_btn_state_;
}

void DeviceTwinButton::retrieveParameters_()
{
  ros::param::get("/twin_button_node/twin_button_velocity", joint_max_spd_);

  ros::param::get("/twin_button_node/twin_up_btn_pin", up_pin_);
  ros::param::get("/twin_button_node/twin_down_btn_pin", down_pin_);
  ros::param::get("/twin_button_node/power_button_pin", power_pin_);
  ros::param::get("/twin_button_node/limit_switch_pin", limit_switch_pin_);
  ros::param::get("/twin_button_node/red_led_pin", red_led_pin_);
  ros::param::get("/twin_button_node/green_led_pin", green_led_pin_);
  ros::param::get("/twin_button_node/blue_led_pin", blue_led_pin_);
  ros::param::get("/twin_button_node/supply_en_pin", supply_en_pin_);
  double duration;
  ros::param::get("/twin_button_node/short_press_duration", duration);
  short_press_duration_ = ros::Duration(duration);
  ros::param::get("/twin_button_node/long_press_duration", duration);
  long_press_duration_ = ros::Duration(duration);

  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
  ROS_DEBUG_NAMED("DeviceTwinButton", "RPI IO settings");
  ROS_DEBUG_NAMED("DeviceTwinButton", "  IO Pin configuration :");
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - up btn       : %d", up_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - down btn     : %d", down_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - power btn    : %d", power_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - limit switch : %d", power_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - red led      : %d", red_led_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - green led    : %d", green_led_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - blue led     : %d", blue_led_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - supply enable: %d", supply_en_pin_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  Speed command :");
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - joint_max_spd_ : %f", joint_max_spd_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
}

void DeviceTwinButton::initializePublishers_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "initializePublishers_");
  cmd_pub_ = n_.advertise<std_msgs::Float64>("/tb_cmd", 1);
}

void DeviceTwinButton::initializeSubscribers_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "initializeSubscribers");
}

void DeviceTwinButton::initializeServices_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "initializeServices");

  power_enable_service_ = n_.advertiseService("/power_enable", &DeviceTwinButton::callbackPowerEnable_, this);
  power_disable_service_ = n_.advertiseService("/power_disable", &DeviceTwinButton::callbackPowerDisable_, this);
  set_led_color_service_ = n_.advertiseService("/led_color", &DeviceTwinButton::callbackSetLedColor_, this);

  ros::service::waitForService("/button_event");
  button_event_srv_ = n_.serviceClient<armms_msgs::ButtonEvent>("/button_event");
}

bool DeviceTwinButton::callbackPowerEnable_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO_NAMED("DeviceTwinButton", "callbackPowerEnable_");
  supply_en_state_ = 1;
  res.success = true;
  res.message = "Power enable";
  return true;
}

bool DeviceTwinButton::callbackPowerDisable_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO_NAMED("DeviceTwinButton", "callbackPowerDisable_");
  supply_en_state_ = 0;
  res.success = true;
  res.message = "Power disable";
  return true;
}

// pwm + blink handling
int DeviceTwinButton::scaleColorPwm_(uint8_t color)
{
  int result = (color * 100) / 255;
  if (result > 100)
  {
    result = 100;
  }
  if (result < 0)
  {
    result = 0;
  }
  return result;
}

bool DeviceTwinButton::callbackSetLedColor_(armms_msgs::SetLedColor::Request& req,
                                            armms_msgs::SetLedColor::Response& res)
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "callbackSetLedColor_");
  led_blink_speed_ = req.blink_speed;

  red_led_state_ = 100 - scaleColorPwm_(req.r);
  green_led_state_ = 100 - scaleColorPwm_(req.g);
  blue_led_state_ = 100 - scaleColorPwm_(req.b);
  return true;
}

void DeviceTwinButton::processInputs_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "processInputs_");

  /* Low side commutation */
  up_btn_state_ = 1 - digitalRead(up_pin_);
  down_btn_state_ = 1 - digitalRead(down_pin_);
  limit_switch_state_ = 1 - digitalRead(limit_switch_pin_);
  /* High side commutation */
  power_btn_state_ = digitalRead(power_pin_);

  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
  ROS_DEBUG_NAMED("DeviceTwinButton", "RPI read input");
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - up btn state       : %d", up_btn_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - down btn state     : %d", down_btn_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - power btn state    : %d", power_btn_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - limit switch state : %d", limit_switch_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
  // debounceButtons_();
}

void DeviceTwinButton::processOutputs_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "processOutputs_");
  // digitalWrite(red_led_pin_, red_led_state_);
  // digitalWrite(green_led_pin_, green_led_state_);
  // digitalWrite(blue_led_pin_, blue_led_state_);
  if (led_blink_speed_ == 0)
  {
    led_blink_state_ = true;
  }
  else
  {
    if (led_blink_counter_ > led_blink_speed_)
    {
      led_blink_counter_ = 0;
      led_blink_state_ = !led_blink_state_;
    }
  }
  led_blink_counter_++;
  if (led_blink_state_)
  {
    softPwmWrite(red_led_pin_, red_led_state_);
    softPwmWrite(green_led_pin_, green_led_state_);
    softPwmWrite(blue_led_pin_, blue_led_state_);
  }
  else
  {
    softPwmWrite(red_led_pin_, 100);
    softPwmWrite(green_led_pin_, 100);
    softPwmWrite(blue_led_pin_, 100);
  }

  digitalWrite(supply_en_pin_, supply_en_state_);

  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
  ROS_DEBUG_NAMED("DeviceTwinButton", "RPI write output");
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - red led            : %d", red_led_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - green led          : %d", green_led_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - blue led           : %d", blue_led_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "  - supply enable      : %d", supply_en_state_);
  ROS_DEBUG_NAMED("DeviceTwinButton", "---------------------------------------");
}

// TODO should I debounce ?
void DeviceTwinButton::debounceButtons_()
{
  int read_state = digitalRead(up_pin_);
  if (read_state == LOW)
  {
    if (ros::Time::now() > debounce_up_btn_)
    {
      debounce_up_btn_ = ros::Time::now() + ros::Duration(debounce_button_time_);
      up_btn_state_ = 1;
    }
  }
  else
  {
    up_btn_state_ = 0;
  }

  read_state = digitalRead(down_pin_);
  if (read_state == LOW)
  {
    if (ros::Time::now() > debounce_down_btn_)
    {
      debounce_down_btn_ = ros::Time::now() + ros::Duration(debounce_button_time_);
      down_btn_state_ = 1;
    }
  }
  else
  {
    down_btn_state_ = 0;
  }
}

}  // namespace input_device

using namespace input_device;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "device_twin_button");

  DeviceTwinButton device_twin_button;

  return 0;
}
