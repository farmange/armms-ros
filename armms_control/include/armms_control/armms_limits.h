//============================================================================
// Name        : armms_limits.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_CONTROL_ARMMS_LIMITS_H
#define ARMMS_CONTROL_ARMMS_LIMITS_H

#include <ros/ros.h>

#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

namespace armms_control
{
class ArmmsLimits
{
public:
  ArmmsLimits(const double& joint_position);
  void saturatePosition(double& cmd);
  void adaptVelocityNearLimits(double& cmd);
  void publishLimits();

private:
  const double& joint_position_;

  ros::NodeHandle nh_;

  ros::ServiceServer set_upper_limit_service_;
  ros::ServiceServer set_lower_limit_service_;
  ros::ServiceServer reset_upper_limit_service_;
  ros::ServiceServer reset_lower_limit_service_;
  ros::ServiceServer enable_upper_limit_service_;
  ros::ServiceServer enable_lower_limit_service_;

  ros::Publisher upper_limit_pub_;
  ros::Publisher lower_limit_pub_;

  bool enable_lower_limit_;
  bool enable_upper_limit_;
  double joint_max_speed_;
  double reduced_speed_divisor_;

  std_msgs::Float64 upper_limit_;
  std_msgs::Float64 lower_limit_;

  void retrieveParameters_();
  void initializeServices_();
  void initializePublishers_();
  void initializeLimits_();

  bool callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackEnableUpperLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableLowerLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  void setUpperLimit_(const float upper_limit_value);
  void setLowerLimit_(const float lower_limit_value);
};

}  // namespace armms_control
#endif
