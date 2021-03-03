//============================================================================
// Name        : armms_at1x_control.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_CONTROL_ARMMS_AT1X_CONTROL_H
#define ARMMS_CONTROL_ARMMS_AT1X_CONTROL_H

#include <ros/ros.h>

#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/SetMotorPower.h"
#include "armms_msgs/ButtonEvent.h"

#include "armms_control/at1x_fsm.h"
#include "armms_control/fsm/engine.h"
#include "armms_control/fsm/state.h"
#include "armms_control/fsm/transition.h"

namespace armms_control
{
class ArmmsAT1XControl
{
public:
  ArmmsAT1XControl();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient set_led_color_srv_;
  ros::ServiceClient set_motor_power_srv_;
  ros::ServiceServer pwr_btn_ev_service_;

  ros::ServiceServer set_upper_limit_service_;
  ros::ServiceServer set_lower_limit_service_;
  ros::ServiceServer reset_upper_limit_service_;
  ros::ServiceServer reset_lower_limit_service_;
  ros::ServiceServer enable_upper_limit_service_;
  ros::ServiceServer enable_lower_limit_service_;

  ros::Publisher position_command_pub_;
  ros::Publisher upper_limit_pub_;
  ros::Publisher lower_limit_pub_;

  ros::Subscriber joint_state_sub_;
  ros::Subscriber user_btn_down_sub_;
  ros::Subscriber user_btn_up_sub_;
  ros::Subscriber switch_limit_sub_;
  ros::Subscriber motor_power_sub_;
  bool refresh_joint_state_;
  bool enable_lower_limit_;
  bool enable_upper_limit_;
  double joint_max_speed_;
  double reduced_speed_divisor_;
  std_msgs::Float64 cmd_;
  std_msgs::Float64 upper_limit_;
  std_msgs::Float64 lower_limit_;
  ros::Timer non_realtime_loop_;
  int sampling_frequency_;
  double sampling_period_;
  double speed_setpoint_;
  double joint_position_;
  ros::Time joint_position_time_;

  bool user_btn_down_;
  bool user_btn_up_;
  bool switch_limit_;
  bool motor_power_;

  bool enable_upper_limit;
  bool enable_lower_limit;
  /* FSM engine */
  Engine<ArmmsAT1XControl>* engine_;

  /* FSM state */
  State<ArmmsAT1XControl>* state_uninitialized_;
  State<ArmmsAT1XControl>* state_stopped_;
  State<ArmmsAT1XControl>* state_position_control_;
  State<ArmmsAT1XControl>* state_error_processing_;
  State<ArmmsAT1XControl>* state_finalized_;

  /* FSM Transitions */
  Transition<ArmmsAT1XControl>* tr_initialized_;
  Transition<ArmmsAT1XControl>* tr_error_raised_;
  Transition<ArmmsAT1XControl>* tr_to_position_control_;
  Transition<ArmmsAT1XControl>* tr_finalize_;
  Transition<ArmmsAT1XControl>* tr_error_success_;
  Transition<ArmmsAT1XControl>* tr_error_critical_;
  Transition<ArmmsAT1XControl>* tr_stop_;

  bool trInitialized_();
  bool trErrorRaised_();
  bool trToPositionControl_();
  bool trFinalize_();
  bool trErrorSuccess_();
  bool trErrorCritical_();
  bool trStop_();

  /* FSM functions */
  void uninitializedEnter_();
  void stoppedEnter_();
  void positionControlEnter_();
  void positionControlUpdate_();
  void errorProcessingEnter_();
  void errorProcessingUpdate_();
  void finalizedEnter_();

  /* FSM input event (what is allowed to do from user point of view) */
  FsmInputEvent input_event_requested_;
  void initializeStateMachine_();
  typedef enum status_e
  {
    OK = 0,
    ERROR
  } status_t;
  status_t status_;

  void retrieveParameters_();
  void initializeServices_();
  void initializeSubscribers_();
  void initializePublishers_();

  void update_(const ros::TimerEvent&);
  void callbackJointStates_(const sensor_msgs::JointStatePtr& msg);
  void callbackUserBtnDown_(const std_msgs::BoolPtr& msg);
  void callbackUserBtnUp_(const std_msgs::BoolPtr& msg);
  void callbackSwitchLimit_(const std_msgs::BoolPtr& msg);
  void callbackMotorPower_(const std_msgs::BoolPtr& msg);
  bool callbackPowerButtonEvent_(armms_msgs::ButtonEvent::Request& req, armms_msgs::ButtonEvent::Response& res);

  bool callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackEnableUpperLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableLowerLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  void init_();
  status_t startMotor_();
  status_t stopMotor_();
  status_t setLedColor_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed);
  void handleLimits_(double& cmd);
  void adaptVelocityNearLimits_(double& cmd, const float divisor);
  void updateJointStates_(std_msgs::Float64& joint_limit, const float joint_value);
  void setUpperLimit_(const float upper_limit_value);
  void setLowerLimit_(const float lower_limit_value);
};

}  // namespace armms_control
#endif
