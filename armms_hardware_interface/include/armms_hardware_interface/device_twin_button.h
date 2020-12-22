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

namespace input_device
{
class DeviceTwinButton
{
public:
  DeviceTwinButton();

private:
  ros::NodeHandle n_;
  ros::Publisher cmd_pub_;

  ros::Time debounce_button_up_;
  ros::Time debounce_button_down_;

  int button_up_;
  int button_down_;

  int up_pin_;
  int down_pin_;

  double velocity_factor_;

  double debounce_button_time_;
  double velocity_factor_inc_;

  double joint_max_speed_;

  void retrieveParameters_();
  void processButtons_();
  void debounceButtons_();
};
}  // namespace input_device
#endif
