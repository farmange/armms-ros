//============================================================================
// Name        : armms_user_input.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_control/armms_user_input.h"

namespace armms_control
{
ArmmsUserInput::ArmmsUserInput()
{
  init_();
  retrieveParameters_();
  initializeServices_();
  initializeSubscribers_();

  if (joint_default_velocity_ > joint_max_velocity_)
  {
    joint_default_velocity_ = joint_max_velocity_;
  }
  joint_setpoint_velocity_ = joint_default_velocity_;

  input_event_requested_ = FsmInputEvent::None;
}

void ArmmsUserInput::initializeServices_()
{
  ROS_DEBUG_NAMED("ArmmsUserInput", "initializeServices_");

  pwr_btn_ev_service_ =
      nh_.advertiseService("/armms_rpi/power_button_event", &ArmmsUserInput::callbackPowerButtonEvent_, this);
}

void ArmmsUserInput::initializeSubscribers_()
{
  ROS_DEBUG_NAMED("ArmmsUserInput", "initializeSubscribers_");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/user_button_down");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/user_button_up");

  user_btn_down_sub_ = nh_.subscribe("/armms_rpi/user_button_down", 1, &ArmmsUserInput::callbackUserBtnDown_, this);
  user_btn_up_sub_ = nh_.subscribe("/armms_rpi/user_button_up", 1, &ArmmsUserInput::callbackUserBtnUp_, this);
  user_setpoint_velocity_sub_ =
      nh_.subscribe("/armms_control/velocity_setpoint", 1, &ArmmsUserInput::callbackVelocitySetpoint_, this);
}

void ArmmsUserInput::retrieveParameters_()
{
  ROS_DEBUG_NAMED("ArmmsUserInput", "retrieveParameters_");
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_velocity_);
  ros::param::get("~/default_velocity", joint_default_velocity_);
}

void ArmmsUserInput::callbackUserBtnDown_(const std_msgs::BoolPtr& msg)
{
  /* Detect rising edge of the button to reset local joint position */
  if (!user_btn_down_ && msg->data)
  {
    refresh_joint_state_ = true;
  }
  user_btn_down_ = msg->data;
}

void ArmmsUserInput::callbackUserBtnUp_(const std_msgs::BoolPtr& msg)
{
  /* Detect rising edge of the button to reset local joint position */
  if (!user_btn_up_ && msg->data)
  {
    refresh_joint_state_ = true;
  }
  user_btn_up_ = msg->data;
}

void ArmmsUserInput::callbackVelocitySetpoint_(const std_msgs::Float64Ptr& msg)
{
  joint_setpoint_velocity_ = msg->data;
  if (joint_setpoint_velocity_ > joint_max_velocity_)
  {
    joint_setpoint_velocity_ = joint_max_velocity_;
  }
}

bool ArmmsUserInput::callbackPowerButtonEvent_(armms_msgs::ButtonEvent::Request& req,
                                               armms_msgs::ButtonEvent::Response& res)
{
  ROS_INFO_NAMED("ArmmsUserInput", "callbackPowerButtonEvent_");
  if (req.button_event == 1)
  {
    /* Short press */
    input_event_requested_ = FsmInputEvent::ButtonShortPress;
  }
  else if (req.button_event == 2)
  {
    /* Long press */
    input_event_requested_ = FsmInputEvent::ButtonLongPress;
  }
  else
  {
    ROS_ERROR_NAMED("ArmmsUserInput", "Button event not supported (%d)", req.button_event);
    return false;
  }
  return true;
}

void ArmmsUserInput::init_()
{
  speed_setpoint_ = NAN;
}

FsmInputEvent ArmmsUserInput::getUserInput()
{
  return input_event_requested_;
}

double ArmmsUserInput::getVelocityCommand()
{
  return velocity_command_;
}

bool ArmmsUserInput::resetJointStateRequest()
{
  return refresh_joint_state_;
}

void ArmmsUserInput::processUserInput()
{
  if (user_btn_up_ && user_btn_down_)
  {
    /* do nothing */
    velocity_command_ = 0.0;
    ROS_INFO_NAMED("ArmmsUserInput", "Conflicting up and down button pressed at the same time !");
  }
  else if (user_btn_up_)
  {
    velocity_command_ = +joint_setpoint_velocity_;
    ROS_INFO_NAMED("ArmmsUserInput", "velocity command received from up button : %f", velocity_command_);
  }
  else if (user_btn_down_)
  {
    velocity_command_ = -joint_setpoint_velocity_;
    ROS_INFO_NAMED("ArmmsUserInput", "velocity command received from down button : %f", velocity_command_);
  }
  else
  {
    velocity_command_ = 0.0;
    ROS_INFO_NAMED("ArmmsUserInput", "no velocity command received !");
  }
}

void ArmmsUserInput::clearUserInput()
{
  refresh_joint_state_ = false;
  input_event_requested_ = FsmInputEvent::None;
}

}  // namespace armms_control