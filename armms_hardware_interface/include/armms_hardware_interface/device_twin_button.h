/*
 *  device_freejoy.h
 *  Copyright (C) 2019 Orthopus
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
#ifndef HL_CONTROLLER_DEVICE_TWIN_BUTTON_H
#define HL_CONTROLLER_DEVICE_TWIN_BUTTON_H

#include <ros/ros.h>

#include "sensor_msgs/Joy.h"
#include "std_msgs/ColorRGBA.h"

namespace input_device
{
class DeviceTwinButton
{
public:
  DeviceTwinButton();

private:
  ros::NodeHandle n_;
  ros::Publisher cmd_pub_;
  ros::Subscriber led_color_sub_;
  ros::ServiceClient start_srv_;
  ros::ServiceClient shutdown_srv_;

  ros::Time debounce_up_btn_;
  ros::Time debounce_down_btn_;
  ros::Time debounce_power_btn_;

  int up_btn_state_;
  int down_btn_state_;
  int power_btn_state_;
  int power_btn_prev_state_;
  int limit_switch_state_;

  int red_led_state_;
  int green_led_state_;
  int blue_led_state_;

  int up_pin_;
  int down_pin_;
  int power_pin_;
  int limit_switch_pin_;

  int red_led_pin_;
  int green_led_pin_;
  int blue_led_pin_;

  double debounce_button_time_;

  double joint_max_spd_;

  void retrieveParameters_();
  void processButtons_();
  void processLeds_();
  void debounceButtons_();
  void initializePublishers_();
  void initializeSubscribers_();
  void initializeServices_();
  void callbackLedColor_(const std_msgs::ColorRGBA& msg);
};
}  // namespace input_device
#endif
