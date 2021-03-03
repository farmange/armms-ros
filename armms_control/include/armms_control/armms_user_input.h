//============================================================================
// Name        : armms_user_input.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description :
//============================================================================

#ifndef ARMMS_CONTROL_ARMMS_USER_INPUT_H
#define ARMMS_CONTROL_ARMMS_USER_INPUT_H

#include <ros/ros.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "armms_msgs/ButtonEvent.h"

#include "armms_control/armms_user_input.h"
#include "armms_control/at1x_fsm.h"

namespace armms_control
{
class ArmmsUserInput
{
public:
  ArmmsUserInput();
  FsmInputEvent getUserInput();
  double getVelocityCommand();
  bool resetJointStateRequest();
  void processUserInput();
  void clearUserInput();

private:
  ros::NodeHandle nh_;

  ros::ServiceServer pwr_btn_ev_service_;

  ros::Subscriber user_btn_down_sub_;
  ros::Subscriber user_btn_up_sub_;
  ros::Subscriber user_setpoint_velocity_sub_;

  bool refresh_joint_state_;
  double joint_max_velocity_;
  double joint_default_velocity_;
  double joint_setpoint_velocity_;
  double speed_setpoint_;
  double velocity_command_;

  bool user_btn_down_;
  bool user_btn_up_;

  FsmInputEvent input_event_requested_;

  void retrieveParameters_();
  void initializeServices_();
  void initializeSubscribers_();

  void callbackUserBtnDown_(const std_msgs::BoolPtr& msg);
  void callbackUserBtnUp_(const std_msgs::BoolPtr& msg);
  void callbackVelocitySetpoint_(const std_msgs::Float64Ptr& msg);

  bool callbackPowerButtonEvent_(armms_msgs::ButtonEvent::Request& req, armms_msgs::ButtonEvent::Response& res);

  void init_();
};

}  // namespace armms_control
#endif
