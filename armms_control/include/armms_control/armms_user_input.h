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
#include "armms_msgs/SetVelocitySetpoint.h"
#include "armms_msgs/RpiInterface.h"
#include "armms_msgs/WebInterface.h"
#include "armms_msgs/IntentInterface.h"

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
  bool getSwitchLimit();
  void processUserInput();
  void clearUserInput();
  void enableUserIntent();
  void disableUserIntent();
  
private:
  ros::NodeHandle nh_;
  ros::ServiceServer pwr_btn_ev_service_;
  ros::ServiceServer setpoint_service_;
  ros::Subscriber rpi_interface_sub_;
  ros::Subscriber web_interface_sub_;
  ros::Subscriber intent_interface_sub_;
  ros::Publisher setpoint_velocity_pub_;

  bool refresh_joint_state_;
  double joint_max_velocity_;
  double joint_default_velocity_;
  double joint_setpoint_velocity_;
  double speed_setpoint_;
  double velocity_command_;

  bool user_btn_down_;
  bool user_btn_up_;
  bool webgui_btn_down_;
  bool webgui_btn_up_;
  bool intent_btn_down_;
  bool intent_btn_up_;
  bool user_intent_enabled_;
  bool switch_limit_;
  FsmInputEvent input_event_requested_;

  void retrieveParameters_();
  void initializeServices_();
  void initializeSubscribers_();
  void initializePublishers_();
  void callbackRpiInterface_(const armms_msgs::RpiInterfacePtr& msg);
  void callbackWebInterface_(const armms_msgs::WebInterfacePtr& msg);
  void callbackIntentInterface_(const armms_msgs::IntentInterfacePtr& msg);
  void callbackVelocitySetpoint_(const std_msgs::Float64Ptr& msg);
  bool callbackPowerButtonEvent_(armms_msgs::ButtonEvent::Request& req, armms_msgs::ButtonEvent::Response& res);
  bool callbackSetVelocitySetpoint_(armms_msgs::SetVelocitySetpoint::Request& req,
                                    armms_msgs::SetVelocitySetpoint::Response& res);

  void init_();
};

}  // namespace armms_control
#endif
