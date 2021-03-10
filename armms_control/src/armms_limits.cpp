//============================================================================
// Name        : armms_limits.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_control/armms_limits.h"

namespace armms_control
{
ArmmsLimits::ArmmsLimits(const double& joint_position)
  : joint_position_(joint_position), enable_upper_limit_(true), enable_lower_limit_(true)
{
  initializeLimits_();
  retrieveParameters_();
  initializeServices_();
  initializePublishers_();

  /* Define a speed divisor, for slow down phases. The value 15.0 could be interpreted as the position in degree where
   * slow down happened. Here, we will start to decrease the speed as soon as we will be under 15 degree from the
   * defined limit.
   */
  reduced_speed_divisor_ = joint_max_speed_ / 15.0;
}

void ArmmsLimits::saturatePosition(double& cmd)
{
  if (upper_limit_.data != NAN && cmd > upper_limit_.data)
  {
    ROS_DEBUG_STREAM_NAMED("ArmmsLimits", "Upper limit overshooted : saturate position command");
    cmd = upper_limit_.data;
  }
  if (lower_limit_.data != NAN && cmd < lower_limit_.data)
  {
    ROS_DEBUG_STREAM_NAMED("ArmmsLimits", "Lower limit overshooted : saturate position command");
    cmd = lower_limit_.data;
  }
}

void ArmmsLimits::adaptVelocityNearLimits(double& cmd)
{
  if (upper_limit_.data != NAN && enable_upper_limit_)
  {
    /* If upper limit is set */
    if (joint_position_ > upper_limit_.data)
    {
      /* If we have exceed the upper limit, we can only go in reverse direction */
      if (cmd > 0.0)
      {
        ROS_DEBUG_NAMED("ArmmsLimits", "Upper limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd > 0.0) && (upper_limit_.data - joint_position_ < (abs(cmd) / reduced_speed_divisor_)))
    {
      ROS_DEBUG_NAMED("ArmmsLimits", "Close to upper limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((upper_limit_.data - joint_position_) / (2.0 * abs(cmd) / reduced_speed_divisor_));
    }
  }

  if (lower_limit_.data != NAN && enable_lower_limit_)
  {
    /* If lower limit is set */
    if (joint_position_ < lower_limit_.data)
    {
      /* If we have exceed the lower limit, we can only go in direct direction */
      if (cmd < 0.0)
      {
        ROS_DEBUG_NAMED("ArmmsLimits", "Lower limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd < 0.0) && (joint_position_ - lower_limit_.data < (abs(cmd) / reduced_speed_divisor_)))
    {
      ROS_DEBUG_NAMED("ArmmsLimits", "Close to lower limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((joint_position_ - lower_limit_.data) / (2.0 * abs(cmd) / reduced_speed_divisor_));
    }
  }
}

void ArmmsLimits::publishLimits()
{
  upper_limit_pub_.publish(upper_limit_);
  lower_limit_pub_.publish(lower_limit_);
}

void ArmmsLimits::initializeServices_()
{
  ROS_DEBUG_NAMED("ArmmsLimits", "initializeServices_");
  set_upper_limit_service_ =
      nh_.advertiseService("/armms_control/set_upper_limit", &ArmmsLimits::callbackSetUpperLimit_, this);
  set_lower_limit_service_ =
      nh_.advertiseService("/armms_control/set_lower_limit", &ArmmsLimits::callbackSetLowerLimit_, this);
  reset_upper_limit_service_ =
      nh_.advertiseService("/armms_control/reset_upper_limit", &ArmmsLimits::callbackResetUpperLimit_, this);
  reset_lower_limit_service_ =
      nh_.advertiseService("/armms_control/reset_lower_limit", &ArmmsLimits::callbackResetLowerLimit_, this);
  enable_upper_limit_service_ =
      nh_.advertiseService("/armms_control/enable_upper_limit", &ArmmsLimits::callbackEnableUpperLimit_, this);
  enable_lower_limit_service_ =
      nh_.advertiseService("/armms_control/enable_lower_limit", &ArmmsLimits::callbackEnableLowerLimit_, this);
}

void ArmmsLimits::initializePublishers_()
{
  ROS_DEBUG_NAMED("ArmmsLimits", "initializePublishers_");
  upper_limit_pub_ = nh_.advertise<std_msgs::Float64>("/armms_control/upper_limit", 1);
  lower_limit_pub_ = nh_.advertise<std_msgs::Float64>("/armms_control/lower_limit", 1);
}

void ArmmsLimits::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsLimits", "retrieveParameters_");
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);
}

void ArmmsLimits::initializeLimits_()
{
  ROS_DEBUG_NAMED("ArmmsLimits", "initializeLimits_");
  upper_limit_.data = NAN;
  lower_limit_.data = NAN;
}

void ArmmsLimits::setUpperLimit_(const float upper_limit_value)
{
  // updateJointStates_(upper_limit_, upper_limit_value);
  upper_limit_.data = joint_position_;
}

void ArmmsLimits::setLowerLimit_(const float lower_limit_value)
{
  // updateJointStates_(lower_limit_, lower_limit_value);
  lower_limit_.data = joint_position_;
}

bool ArmmsLimits::callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackSetUpperLimit_");
  setUpperLimit_(joint_position_);
  return true;
}

bool ArmmsLimits::callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackSetLowerLimit_");
  setLowerLimit_(joint_position_);
  return true;
}

bool ArmmsLimits::callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackResetUpperLimit_");
  setUpperLimit_(NAN);
  return true;
}

bool ArmmsLimits::callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackResetLowerLimit_");
  setLowerLimit_(NAN);
  return true;
}

bool ArmmsLimits::callbackEnableUpperLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackEnableUpperLimit_");
  enable_upper_limit_ = req.data;
  res.success = true;
  res.message = "Upper limit correctly enabled";
  return true;
}

bool ArmmsLimits::callbackEnableLowerLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  ROS_INFO_NAMED("ArmmsLimits", "callbackEnableLowerLimit_");
  enable_lower_limit_ = req.data;
  res.success = true;
  res.message = "Lower limit correctly enabled";
  return true;
}
}  // namespace armms_control
