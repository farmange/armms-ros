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

namespace input_device
{
DeviceTwinButton::DeviceTwinButton()
{
  cmd_pub_ = n_.advertise<std_msgs::Float64>("/tb_cmd", 1);
  retrieveParameters_();

  debounce_button_up_ = ros::Time::now();
  debounce_button_down_ = ros::Time::now();

  button_up_ = 0;
  button_down_ = 0;

  velocity_factor_ = joint_max_speed_;

  debounce_button_time_ = 0.2;
  velocity_factor_inc_ = 1;
  if (wiringPiSetup() == -1)
  {
    ROS_ERROR("wiringPiSetup error !");
    exit(0);
  }
  pinMode(1, OUTPUT);
  pinMode(up_pin_, INPUT);
  pinMode(down_pin_, INPUT);
  pullUpDnControl(up_pin_, PUD_UP);
  pullUpDnControl(down_pin_, PUD_UP);
  ros::Rate loop_rate = ros::Rate(100);

  ROS_DEBUG("up_pin_ = %d", up_pin_);
  ROS_DEBUG("down_pin_ = %d", down_pin_);
  ROS_DEBUG("joint_max_speed_ = %f", joint_max_speed_);
  int state = 0;
  while (1)
  {
    ros::spinOnce();

    digitalWrite(1, state);

    processButtons_();
    std_msgs::Float64 joint_vel;
    ROS_DEBUG("button_up_ = %d", button_up_);
    ROS_DEBUG("button_down_ = %d", button_down_);
    if (button_up_ == 1 && button_down_ == 0)
    {
      joint_vel.data = +joint_max_speed_;
    }
    else if (button_up_ == 0 && button_down_ == 1)
    {
      joint_vel.data = -joint_max_speed_;
    }
    else
    {
      joint_vel.data = 0;
    }

    cmd_pub_.publish(joint_vel);
    loop_rate.sleep();
  }
}

void DeviceTwinButton::retrieveParameters_()
{
  // ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);
  ros::param::get("/twin_button_node/twin_button_velocity", joint_max_speed_);
  // ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);

  ros::param::get("/twin_button_node/twin_button_up_pin", up_pin_);
  ros::param::get("/twin_button_node/twin_button_down_pin", down_pin_);
}

void DeviceTwinButton::processButtons_()
{
  // button_up_ = 0;
  // button_down_ = 0;
  button_up_ = 1 - digitalRead(up_pin_);
  button_down_ = 1 - digitalRead(down_pin_);
  // debounceButtons_();
}

void DeviceTwinButton::debounceButtons_()
{
  int read_state = digitalRead(up_pin_);
  if (read_state == LOW)
  {
    if (ros::Time::now() > debounce_button_up_)
    {
      debounce_button_up_ = ros::Time::now() + ros::Duration(debounce_button_time_);
      button_up_ = 1;
    }
  }
  else
  {
    button_up_ = 0;
  }

  read_state = digitalRead(down_pin_);
  if (read_state == LOW)
  {
    if (ros::Time::now() > debounce_button_down_)
    {
      debounce_button_down_ = ros::Time::now() + ros::Duration(debounce_button_time_);
      button_down_ = 1;
    }
  }
  else
  {
    button_down_ = 0;
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
