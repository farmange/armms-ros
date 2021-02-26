/*
 *  hl_controller.h
 *  Copyright (C) 2019 Orthopus
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
#ifndef HL_CONTROLLER_H
#define HL_CONTROLLER_H

#include <ros/ros.h>
// #include "kinova_msgs/JointAngles.h"
// #include "kinova_msgs/JointVelocity.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/ButtonEvent.h"

#include "armms_hardware_interface/robot_manager_fsm.h"
#include "armms_hardware_interface/fsm/engine.h"
#include "armms_hardware_interface/fsm/state.h"
#include "armms_hardware_interface/fsm/transition.h"

// TODO change namespace
namespace orthopus_addon
{
class HLController
{
public:
  HLController();

private:
  ros::NodeHandle n_;
  ros::Subscriber joint_angles_sub_;
  ros::Subscriber joy_cmd_sub_;
  ros::Subscriber gui_cmd_sub_;
  ros::Subscriber tb_cmd_sub_;
  ros::Subscriber spd_setpoint_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher upper_limit_pub_;
  ros::Publisher lower_limit_pub_;
  ros::Publisher joint_angle_pub_;

  ros::ServiceServer button_event_service_;
  ros::ServiceServer set_upper_limit_service_;
  ros::ServiceServer set_lower_limit_service_;
  ros::ServiceServer reset_upper_limit_service_;
  ros::ServiceServer reset_lower_limit_service_;
  ros::ServiceServer enable_upper_limit_service_;
  ros::ServiceServer enable_lower_limit_service_;

  ros::ServiceClient switch_ctrl_srv_;
  ros::ServiceClient power_enable_srv_;
  ros::ServiceClient power_disable_srv_;
  ros::ServiceClient start_motor_control_srv_;
  ros::ServiceClient stop_motor_control_srv_;
  ros::ServiceClient set_led_color_srv_;

  /* FSM engine */
  Engine<HLController>* engine_;

  /* FSM state */
  State<HLController>* state_uninitialized_;
  State<HLController>* state_stopped_;
  State<HLController>* state_position_control_;
  State<HLController>* state_error_processing_;
  State<HLController>* state_finalized_;

  /* FSM Transitions */
  Transition<HLController>* tr_initialized_;
  Transition<HLController>* tr_error_raised_;
  Transition<HLController>* tr_to_position_control_;
  Transition<HLController>* tr_finalize_;
  Transition<HLController>* tr_error_success_;
  Transition<HLController>* tr_error_critical_;
  Transition<HLController>* tr_stop_;

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
  typedef enum status_e
  {
    OK = 0,
    ERROR
  } status_t;
  status_t status_;

  int sampling_freq_;
  int direction_;
  double joint_max_speed_;
  double sampling_period_;
  bool enable_upper_limit;
  bool enable_lower_limit;
  bool close_loop_control_;
  double prev_pos_;
  double speed_setpoint_;
  float reduced_speed_divisor_;

  std_msgs::Float64 joint_angles_;
  std_msgs::Float64 upper_limit_;
  std_msgs::Float64 lower_limit_;
  std_msgs::Float64 joy_cmd_;
  std_msgs::Float64 tb_cmd_;
  std_msgs::Float64 gui_cmd_;
  std_msgs::Float64 cmd_;
  bool refresh_joint_state_;
  void initializeSubscribers_();
  void initializePublishers_();
  void initializeServices_();
  void retrieveParameters_();
  void initializeStateMachine_();
  void init_();

  status_t startMotor_();
  status_t stopMotor_();
  status_t setLedColor_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed);

  void handleLimits_(double& cmd);
  void adaptVelocityNearLimits_(double& cmd, const float divisor);

  void updateJointStates_(std_msgs::Float64& joint_angles, int joint_number, float joint_value);
  void setUpperLimit_(const int joint_number, const float upper_limit_value);
  void setLowerLimit_(const int joint_number, const float upper_limit_value);

  void callbackJointStates_(const sensor_msgs::JointStatePtr& msg);
  void callbackJoyCmd_(const std_msgs::Float64Ptr& msg);
  void callbackGuiCmd_(const std_msgs::Float64Ptr& msg);
  void callbackTbCmd_(const std_msgs::Float64Ptr& msg);

  bool callbackButtonEvent_(armms_msgs::ButtonEvent::Request& req, armms_msgs::ButtonEvent::Response& res);

  void callbackSpeedSetpoint_(const std_msgs::Float64Ptr& msg);
  bool callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackEnableUpperLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackEnableLowerLimit_(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
};
}  // namespace orthopus_addon
#endif
