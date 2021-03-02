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
#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/ButtonEvent.h"
#include "std_srvs/Trigger.h"

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
  ros::ServiceClient button_event_srv_;
  ros::ServiceServer power_enable_service_;
  ros::ServiceServer power_disable_service_;
  ros::ServiceServer set_led_color_service_;

  ros::Time debounce_up_btn_;
  ros::Time debounce_down_btn_;
  ros::Time debounce_power_btn_;

  ros::Timer non_realtime_loop_;

  // TODO improve io definition with struct { mode, state, desired_state, pin } */

  /* Input states */
  int up_btn_state_;
  int down_btn_state_;
  int power_btn_state_;
  int power_btn_prev_state_;
  int limit_switch_state_;

  typedef enum button_action_e
  {
    BTN_NONE = 0,
    BTN_SHORT_PRESS,
    BTN_LONG_PRESS
  } button_action_t;
  button_action_t button_action_;

  /* Output states */
  int red_led_state_;
  int green_led_state_;
  int blue_led_state_;
  int supply_en_state_;
  uint8_t led_blink_speed_;
  uint8_t led_blink_counter_;
  bool led_blink_state_;
  /* Input pins */
  int up_pin_;
  int down_pin_;
  int power_pin_;
  int limit_switch_pin_;

  const int sampling_frequency_ = 100;
  ros::Duration short_press_duration_;
  ros::Duration long_press_duration_;
  ros::Time press_time_;

  /* Output pins */
  int red_led_pin_;
  int green_led_pin_;
  int blue_led_pin_;
  int supply_en_pin_;

  double debounce_button_time_;

  double joint_max_spd_;
  void update_(const ros::TimerEvent& e);

  void retrieveParameters_();
  void processInputs_();
  void processOutputs_();
  void debounceButtons_();
  int scaleColorPwm_(uint8_t color);

  void initializePublishers_();
  void initializeSubscribers_();
  void initializeServices_();
  bool callbackPowerEnable_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackPowerDisable_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackSetLedColor_(armms_msgs::SetLedColor::Request& req, armms_msgs::SetLedColor::Response& res);
};
}  // namespace input_device
#endif
