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
  initializePublishers_();

  if (joint_default_velocity_ > joint_max_velocity_)
  {
    joint_default_velocity_ = joint_max_velocity_;
  }
  joint_setpoint_velocity_ = joint_default_velocity_;

  input_event_requested_ = FsmInputEvent::None;
}

void ArmmsUserInput::initializeServices_()
{
  pwr_btn_ev_service_ =
      nh_.advertiseService("/armms_rpi/power_button_event", &ArmmsUserInput::callbackPowerButtonEvent_, this);
  setpoint_service_ =
      nh_.advertiseService("/armms_control/set_velocity_setpoint", &ArmmsUserInput::callbackSetVelocitySetpoint_, this);
}

void ArmmsUserInput::initializeSubscribers_()
{
  ros::topic::waitForMessage<armms_msgs::RpiInterface>("/armms_rpi/rpi_interface");

  rpi_interface_sub_ = nh_.subscribe("/armms_rpi/rpi_interface", 1, &ArmmsUserInput::callbackRpiInterface_, this);
  web_interface_sub_ = nh_.subscribe("/armms_web/web_interface", 1, &ArmmsUserInput::callbackWebInterface_, this);
  intent_interface_sub_ =
      nh_.subscribe("/armms_user_intent/intent_interface", 1, &ArmmsUserInput::callbackIntentInterface_, this);
}

void ArmmsUserInput::initializePublishers_()
{
  setpoint_velocity_pub_ = nh_.advertise<std_msgs::Float64>("/armms_control/velocity_setpoint", 1);
}

void ArmmsUserInput::retrieveParameters_()
{
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_velocity_);
  ros::param::get("~/default_velocity", joint_default_velocity_);
}

void ArmmsUserInput::callbackRpiInterface_(const armms_msgs::RpiInterfacePtr& msg)
{
  /* Detect rising edge of the down button to reset local joint position */
  if (!user_btn_down_ && msg->user_button_down)
  {
    refresh_joint_state_ = true;
  }
  user_btn_down_ = msg->user_button_down;

  /* Detect rising edge of the up button to reset local joint position */
  if (!user_btn_up_ && msg->user_button_up)
  {
    refresh_joint_state_ = true;
  }
  user_btn_up_ = msg->user_button_up;

  switch_limit_ = msg->switch_limit;
}

void ArmmsUserInput::callbackWebInterface_(const armms_msgs::WebInterfacePtr& msg)
{
  /* Detect rising edge of the down button to reset local joint position */
  if (!webgui_btn_down_ && msg->user_button_down)
  {
    refresh_joint_state_ = true;
  }
  webgui_btn_down_ = msg->user_button_down;

  /* Detect rising edge of the up button to reset local joint position */
  if (!webgui_btn_up_ && msg->user_button_up)
  {
    refresh_joint_state_ = true;
  }
  webgui_btn_up_ = msg->user_button_up;
}

void ArmmsUserInput::callbackIntentInterface_(const armms_msgs::IntentInterfacePtr& msg)
{
  /* Detect rising edge of the down button to reset local joint position */
  if (!intent_btn_down_ && msg->user_button_down)
  {
    refresh_joint_state_ = true;
  }
  intent_btn_down_ = msg->user_button_down;

  /* Detect rising edge of the up button to reset local joint position */
  if (!intent_btn_up_ && msg->user_button_up)
  {
    refresh_joint_state_ = true;
  }
  intent_btn_up_ = msg->user_button_up;
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
  if (req.button_event == 1)
  {
    /* Short press */
    input_event_requested_ = FsmInputEvent::ButtonShortPress;
  }
  else if (req.button_event == 2)
  {
    /* Short double press */
    input_event_requested_ = FsmInputEvent::ButtonShortDoublePress;
  }
  else if (req.button_event == 3)
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

bool ArmmsUserInput::callbackSetVelocitySetpoint_(armms_msgs::SetVelocitySetpoint::Request& req,
                                                  armms_msgs::SetVelocitySetpoint::Response& res)
{
  if (req.value < 0)
  {
    ROS_WARN_NAMED("ArmmsUserInput", "Negative velocity setpoint are not allowed (value:%f)", req.value);
    return false;
  }
  else
  {
    if (req.value > joint_max_velocity_)
    {
      joint_setpoint_velocity_ = joint_max_velocity_;
      ROS_WARN_NAMED("ArmmsUserInput",
                     "Velocity setpoint (%f) is bigger than max velocity allowed (%f) not allowed. Value "
                     "requested will be saturated !",
                     req.value, joint_max_velocity_);
    }
    else
    {
      joint_setpoint_velocity_ = req.value;
    }
  }
  return true;
}

void ArmmsUserInput::init_()
{
  speed_setpoint_ = NAN;
  user_btn_down_ = false;
  user_btn_up_ = false;
  webgui_btn_down_ = false;
  webgui_btn_up_ = false;
  intent_btn_down_ = false;
  intent_btn_up_ = false;
  user_intent_enabled_ = false;
}

FsmInputEvent ArmmsUserInput::getUserInput()
{
  return input_event_requested_;
}

double ArmmsUserInput::getVelocityCommand()
{
  bool dummy;
  return getVelocityCommand(dummy);
}

double ArmmsUserInput::getVelocityCommand(bool& is_intent_command)
{
  is_intent_command = is_intent_command_;
  return velocity_command_;
}

bool ArmmsUserInput::getSwitchLimit()
{
  return switch_limit_;
}

bool ArmmsUserInput::resetJointStateRequest()
{
  return refresh_joint_state_;
}

void ArmmsUserInput::processUserInput()
{
  is_intent_command_ = false;
  if (user_btn_up_ && user_btn_down_)
  {
    /* do nothing */
    velocity_command_ = 0.0;
    ROS_DEBUG_NAMED("ArmmsUserInput", "Conflicting up and down button pressed at the same time !");
  }
  else if (user_btn_up_)
  {
    velocity_command_ = +joint_setpoint_velocity_;
    ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from up button : %f", velocity_command_);
  }
  else if (user_btn_down_)
  {
    velocity_command_ = -joint_setpoint_velocity_;
    ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from down button : %f", velocity_command_);
  }
  else if (webgui_btn_up_ && webgui_btn_down_)
  {
    /* do nothing */
    velocity_command_ = 0.0;
    ROS_DEBUG_NAMED("ArmmsUserInput", "Conflicting webgui up and down button pressed at the same time !");
  }
  else if (webgui_btn_up_)
  {
    velocity_command_ = +joint_setpoint_velocity_;
    ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from webgui up button : %f", velocity_command_);
  }
  else if (webgui_btn_down_)
  {
    velocity_command_ = -joint_setpoint_velocity_;
    ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from webgui down button : %f", velocity_command_);
  }
  else if (user_intent_enabled_)
  {
    if (intent_btn_up_ && intent_btn_down_)
    {
      /* do nothing */
      velocity_command_ = 0.0;
      ROS_DEBUG_NAMED("ArmmsUserInput", "Conflicting user intent up and down button pressed at the same time !");
    }
    else if (intent_btn_up_)
    {
      is_intent_command_ = true;
      velocity_command_ = +joint_setpoint_velocity_;
      ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from user intent up button : %f", velocity_command_);
    }
    else if (intent_btn_down_)
    {
      is_intent_command_ = true;
      velocity_command_ = -joint_setpoint_velocity_;
      ROS_DEBUG_NAMED("ArmmsUserInput", "velocity command received from user intent down button : %f",
                      velocity_command_);
    }
    else
    {
      velocity_command_ = 0.0;
      ROS_DEBUG_NAMED("ArmmsUserInput", "no velocity command received !");
    }
  }
  else
  {
    velocity_command_ = 0.0;
    ROS_DEBUG_NAMED("ArmmsUserInput", "no velocity command received !");
  }

  /* publish velocity setpoint */
  std_msgs::Float64 msg;
  msg.data = joint_setpoint_velocity_;
  setpoint_velocity_pub_.publish(msg);
}

void ArmmsUserInput::clearUserInput()
{
  refresh_joint_state_ = false;
  input_event_requested_ = FsmInputEvent::None;
}

void ArmmsUserInput::enableUserIntent()
{
  user_intent_enabled_ = true;
}

void ArmmsUserInput::disableUserIntent()
{
  user_intent_enabled_ = false;
}

}  // namespace armms_control