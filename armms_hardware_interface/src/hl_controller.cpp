/*
 *  hl_controller.cpp
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
#include <cmath>
#include "ros/ros.h"

#include <armms_hardware_interface/hl_controller.h>

/**
 * Warning : this package is only compatible with one joint (joint1) AT1X device
 */
namespace orthopus_addon
{
HLController::HLController() : sampling_freq_(0.0), direction_(1), enable_upper_limit(true), enable_lower_limit(true)
{
  init_();
  retrieveParameters_();
  initializePublishers_();
  initializeSubscribers_();
  initializeServices_();

  if (sampling_freq_ > 0)
  {
    // TODO unused sampling period
    sampling_period_ = 1.0 / sampling_freq_;
  }
  else
  {
    ROS_ERROR("Sampling frequency could not be lower or equal to zero : %d", sampling_freq_);
    ros::shutdown();
  }
  ros::Rate loop_rate = ros::Rate(sampling_freq_);
  ROS_INFO("Sampling frequency : %d", sampling_freq_);

  /* Define a speed divisor, for slow down phases. The value 15.0 could be interpreted as the position in degree where
   * slow down happened. Here, we will start to decrease the speed as soon as we will be under 15 degree from the
   * defined limit.
   */
  const float reduced_speed_divisor = joint_max_speed_ / 15.0;

  while (ros::ok())
  {
    ros::spinOnce();
    // ROS_DEBUG_STREAM_NAMED("HLController", "Joint angles :\n" << joint_angles_);

    upper_limit_pub_.publish(upper_limit_);
    lower_limit_pub_.publish(lower_limit_);
    joint_angle_pub_.publish(joint_angles_);

    std_msgs::Float64 cmd;

    /* Prioritize the input velocity command. GUI is higher priority so we take joypad command only
     * if no GUI command is received */
    if (gui_cmd_.data != 0.0)
    {
      cmd = gui_cmd_;
    }
    else
    {
      cmd = joy_cmd_;
    }

    // handleLimits_(cmd, reduced_speed_divisor);
    cmd.data *= direction_;

    cmd.data = joint_angles_.data + (cmd.data * 10 * sampling_period_);
    cmd_pub_.publish(cmd);
    loop_rate.sleep();
  }
}

void HLController::initializeSubscribers_()
{
  ROS_DEBUG_NAMED("HLController", "initializeSubscribers");
  // TODO get robotname from config file instead of fixed string "j2n6s300"
  joint_angles_sub_ = n_.subscribe("/joint_states", 1, &HLController::callbackJointStates_, this);
  joy_cmd_sub_ = n_.subscribe("/joy_cmd", 1, &HLController::callbackJoyCmd_, this);
  gui_cmd_sub_ = n_.subscribe("/gui_cmd", 1, &HLController::callbackGuiCmd_, this);
}

void HLController::initializePublishers_()
{
  ROS_DEBUG_NAMED("HLController", "initializePublishers");
  cmd_pub_ = n_.advertise<std_msgs::Float64>("/at1x/controller/position/joint1/command", 1);
  upper_limit_pub_ = n_.advertise<std_msgs::Float64>("/upper_limit", 1);
  lower_limit_pub_ = n_.advertise<std_msgs::Float64>("/lower_limit", 1);
  joint_angle_pub_ = n_.advertise<std_msgs::Float64>("/joint_angle", 1);
}

void HLController::initializeServices_()
{
  ROS_DEBUG_NAMED("HLController", "initializeServices");
  set_upper_limit_service_ = n_.advertiseService("/set_upper_limit", &HLController::callbackSetUpperLimit_, this);
  set_lower_limit_service_ = n_.advertiseService("/set_lower_limit", &HLController::callbackSetLowerLimit_, this);
  reset_upper_limit_service_ = n_.advertiseService("/reset_upper_limit", &HLController::callbackResetUpperLimit_, this);
  reset_lower_limit_service_ = n_.advertiseService("/reset_lower_limit", &HLController::callbackResetLowerLimit_, this);
  enable_upper_limit_service_ =
      n_.advertiseService("/enable_upper_limit", &HLController::callbackEnableUpperLimit_, this);
  enable_lower_limit_service_ =
      n_.advertiseService("/enable_lower_limit", &HLController::callbackEnableLowerLimit_, this);
}

void HLController::retrieveParameters_()
{
  ROS_DEBUG_NAMED("HLController", "retrieveParameters");
  ros::param::get("~sampling_frequency", sampling_freq_);
  ros::param::get("~direction", direction_);
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);
}

void HLController::init_()
{
  upper_limit_.data = NAN;
  lower_limit_.data = NAN;
}

void HLController::handleLimits_(std_msgs::Float64& cmd, const float divisor)
{
  if (upper_limit_.data != NAN && enable_upper_limit)
  {
    /* If upper limit is set */
    if (joint_angles_.data > upper_limit_.data)
    {
      /* If we have exceed the upper limit, we can only go in reverse direction */
      if (cmd.data > 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("HLController", "Upper limit overshooted. Cannot move in this direction !");
        cmd.data = 0.0;
      }
    }
    else if ((cmd.data > 0.0) && (upper_limit_.data - joint_angles_.data < (abs(cmd.data) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("HLController", "Close to upper limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd.data *= ((upper_limit_.data - joint_angles_.data) / (2.0 * abs(cmd.data) / divisor));
    }
  }

  if (lower_limit_.data != NAN && enable_lower_limit)
  {
    /* If lower limit is set */
    if (joint_angles_.data < lower_limit_.data)
    {
      /* If we have exceed the lower limit, we can only go in direct direction */
      if (cmd.data < 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("HLController", "Lower limit overshooted. Cannot move in this direction !");
        cmd.data = 0.0;
      }
    }
    else if ((cmd.data < 0.0) && (joint_angles_.data - lower_limit_.data < (abs(cmd.data) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("HLController", "Close to lower limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd.data *= ((joint_angles_.data - lower_limit_.data) / (2.0 * abs(cmd.data) / divisor));
    }
  }
}

void HLController::updateJointStates_(std_msgs::Float64& joint_angles, const int joint_number, const float joint_value)
{
  ROS_DEBUG_NAMED("HLController", "Update joint%d angles with value : %f", joint_number, joint_value);

  switch (joint_number)
  {
    case 1:
      joint_angles.data = joint_value;
      break;
    case 2:
      joint_angles.data = joint_value;
      break;
    case 3:
      joint_angles.data = joint_value;
      break;
    case 4:
      joint_angles.data = joint_value;
      break;
    case 5:
      joint_angles.data = joint_value;
      break;
    case 6:
      joint_angles.data = joint_value;
      break;
    case 7:
      joint_angles.data = joint_value;
      break;
    default:
      break;
  }
}

void HLController::setUpperLimit_(const int joint_number, const float upper_limit_value)
{
  updateJointStates_(upper_limit_, joint_number, upper_limit_value);
}

void HLController::setLowerLimit_(const int joint_number, const float lower_limit_value)
{
  updateJointStates_(lower_limit_, joint_number, lower_limit_value);
}

void HLController::callbackJointStates_(const sensor_msgs::JointStatePtr& msg)
{
  ROS_DEBUG("callbackJointStates_");
  joint_angles_.data = direction_ * msg->position[0];
}

void HLController::callbackJoyCmd_(const std_msgs::Float64Ptr& msg)
{
  joy_cmd_.data = msg->data;
}

void HLController::callbackGuiCmd_(const std_msgs::Float64Ptr& msg)
{
  gui_cmd_.data = msg->data;
}

// TODO improve services to handle multiple joints configuration (e.g. srv could take integer parameter)
bool HLController::callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackSetUpperLimit_");
  setUpperLimit_(1, joint_angles_.data);
  return true;
}

bool HLController::callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackSetLowerLimit_");
  setLowerLimit_(1, joint_angles_.data);
  return true;
}

bool HLController::callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackResetUpperLimit_");
  setUpperLimit_(1, NAN);
  return true;
}

bool HLController::callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackResetLowerLimit_");
  setLowerLimit_(1, NAN);
  return true;
}

bool HLController::callbackEnableUpperLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackEnableUpperLimit_");
  enable_upper_limit = req.data;
  return true;
}

bool HLController::callbackEnableLowerLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackEnableLowerLimit_");
  enable_lower_limit = req.data;
  return true;
}
}  // namespace orthopus_addon

// TODO in another file
using namespace orthopus_addon;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hl_controller");

  HLController hl_controller;

  return 0;
}
