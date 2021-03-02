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
#ifndef HL_CONTROLLER_DEVICE_FREEJOY_H
#define HL_CONTROLLER_DEVICE_FREEJOY_H

#include <ros/ros.h>

#include "sensor_msgs/Joy.h"

namespace input_device
{
class DeviceFreejoy
{
public:
  DeviceFreejoy();

private:
  ros::NodeHandle n_;
  ros::Subscriber device_sub_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient set_upper_limit_srv_;
  ros::ServiceClient set_lower_limit_srv_;
  ros::ServiceClient reset_upper_limit_srv_;
  ros::ServiceClient reset_lower_limit_srv_;

  ros::Time debounce_button_lb_;
  ros::Time debounce_button_la_;
  ros::Time debounce_button_rb_;
  ros::Time debounce_button_ra_;
  ros::Time debounce_button_1_;
  ros::Time debounce_button_3_;
  int button_lb_;
  int button_la_;
  int button_rb_;
  int button_ra_;
  int button_1_;
  int button_3_;
  double velocity_factor_;

  double debounce_button_time_;
  double velocity_factor_inc_;

  double joint_max_speed_;

  void initializeServices_();
  void retrieveParameters_();
  void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                       int& button_value_ptr);
  void updateVelocityFactor();
  void updateLimits();
};
}  // namespace input_device
#endif
