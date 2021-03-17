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
#include "armms_msgs/SetInt.h"
#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/SetMotorPower.h"
#include "armms_msgs/ButtonEvent.h"

#include "armms_control/armms_limits.h"
#include "armms_control/armms_user_input.h"
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
  ArmmsLimits limit_handler_;
  ArmmsUserInput user_input_handler_;
  ros::NodeHandle nh_;
  ros::ServiceClient set_led_color_srv_;
  ros::ServiceClient set_motor_power_srv_;
  ros::ServiceClient shutdown_srv_;
  ros::ServiceClient reset_controller_srv_;
  ros::Publisher position_command_pub_;
  ros::Subscriber joint_state_sub_;
  bool refresh_joint_state_;
  double joint_max_speed_;
  double reduced_speed_divisor_;
  double input_velocity_cmd_;
  std_msgs::Float64 cmd_;
  ros::Timer non_realtime_loop_;
  int loop_rate_;
  double sampling_period_;
  double joint_position_;
  ros::Time joint_position_time_;

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
  void finalizedUpdate_();

  /* FSM input event (what is allowed to do from user point of view) */
  // FsmInputEvent input_event_requested_;
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

  void callbackJointStates_(const sensor_msgs::JointStatePtr& msg);

  status_t startMotor_();
  status_t stopMotor_();
  status_t setLedColor_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed);
  status_t shutdown_();
};

}  // namespace armms_control
#endif
