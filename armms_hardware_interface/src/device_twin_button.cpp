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

#include "armms_hardware_interface/device_twin_button.h"
#include <wiringPi.h>

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

  debounce_button_time_ = 0.2;
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

  pullUpDnControl(up_pin_, PUD_OFF);
  pullUpDnControl(down_pin_, PUD_OFF);
  // TODO check pull up configuration for power button
  pullUpDnControl(power_pin_, PUD_OFF);
  pullUpDnControl(limit_switch_pin_, PUD_OFF);

  ros::Rate loop_rate = ros::Rate(100);

  while (1)
  {
    ros::spinOnce();

    processButtons_();
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
    if (power_btn_state_ == 1 && power_btn_prev_state_ == 0)
    {
      std_srvs::Empty dummy;

      if (!shutdown_srv_.call(dummy))
      {
        ROS_ERROR_NAMED("HLController", "Problem occured during AT1X shutdown");
        /* TODO handle error here */
      }
    }
    power_btn_prev_state_ = power_btn_state_;
    loop_rate.sleep();
  }
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
  led_color_sub_ = n_.subscribe("/rpi_interface/led_color", 1, &DeviceTwinButton::callbackLedColor_, this);
}

void DeviceTwinButton::initializeServices_()
{
  ROS_DEBUG_NAMED("DeviceTwinButton", "initializeServices");
  ros::service::waitForService("/start");
  ros::service::waitForService("/shutdown");
  start_srv_ = n_.serviceClient<std_srvs::Empty>("/start");
  shutdown_srv_ = n_.serviceClient<std_srvs::Empty>("/shutdown");
}

void DeviceTwinButton::callbackLedColor_(const std_msgs::ColorRGBA& msg)
{
  // if (msg->r != 0)
  // {
  //   red_led_state_ = 1;
  // }
  // else
  // {
  //   red_led_state_ = 0;
  // }

  // if (msg->g != 0)
  // {
  //   green_led_state_ = 1;
  // }
  // else
  // {
  //   green_led_state_ = 0;
  // }

  // if (msg->b != 0)
  // {
  //   blue_led_state_ = 1;
  // }
  // else
  // {
  //   blue_led_state_ = 0;
  // }
}

void DeviceTwinButton::processButtons_()
{
  /* Low side commutation */
  up_btn_state_ = 1 - digitalRead(up_pin_);
  down_btn_state_ = 1 - digitalRead(down_pin_);
  power_btn_state_ = 1 - digitalRead(limit_switch_state_);
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

void DeviceTwinButton::processLeds_()
{
  digitalWrite(red_led_pin_, red_led_state_);
  digitalWrite(green_led_pin_, green_led_state_);
  digitalWrite(blue_led_pin_, blue_led_state_);
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
