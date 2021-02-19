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

#include <controller_manager_msgs/SwitchController.h>
// TODO replace all Empty service with Trigger (bool response)
#include <armms_hardware_interface/hl_controller.h>
/**
 * Warning : For now this package is only compatible with the one joint (joint1) device AT1X
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
  initializeStateMachine_();

  if (sampling_freq_ > 0)
  {
    // TODO unused sampling period
    sampling_period_ = 1.0 / sampling_freq_;
  }
  else
  {
    ROS_ERROR_NAMED("HLController", "Sampling frequency could not be lower or equal to zero : %d", sampling_freq_);
    ros::shutdown();
  }
  ros::Rate loop_rate = ros::Rate(sampling_freq_);
  ROS_INFO_NAMED("HLController", "Sampling frequency : %d Hz", sampling_freq_);

  /* Define a speed divisor, for slow down phases. The value 15.0 could be interpreted as the position in degree where
   * slow down happened. Here, we will start to decrease the speed as soon as we will be under 15 degree from the
   * defined limit.
   */
  reduced_speed_divisor_ = joint_max_speed_ / 15.0;

  ROS_INFO_NAMED("HLController", "Waiting for /joint_states messages...");
  ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  ROS_INFO_NAMED("HLController", "/joint_states messages received");
  /* SpinOnce to retrieve joint state */
  ros::spinOnce();

  /* Initialize position command with initial joint position */
  cmd_.data = joint_angles_.data;
  ROS_INFO_NAMED("HLController", "uninitializedEnter_");
  input_event_requested_ = FsmInputEvent::Start;
  while (ros::ok())
  {
    ros::spinOnce();

    /**** FSM ****/
    ROS_DEBUG_NAMED("HLController", "Current state is '%s' and input event is '%s'",
                    engine_->getCurrentState()->getName().c_str(), input_event_requested_.toString().c_str());

    engine_->process();
    input_event_requested_ = FsmInputEvent::None;
    /*************/

    loop_rate.sleep();
  }
}

void HLController::initializeSubscribers_()
{
  ROS_DEBUG_NAMED("HLController", "initializeSubscribers");
  joint_angles_sub_ = n_.subscribe("/joint_states", 1, &HLController::callbackJointStates_, this);
  joy_cmd_sub_ = n_.subscribe("/joy_cmd", 1, &HLController::callbackJoyCmd_, this);
  gui_cmd_sub_ = n_.subscribe("/gui_cmd", 1, &HLController::callbackGuiCmd_, this);
  tb_cmd_sub_ = n_.subscribe("/tb_cmd", 1, &HLController::callbackTbCmd_, this);
  spd_setpoint_sub_ = n_.subscribe("/speed_setpoint", 1, &HLController::callbackSpeedSetpoint_, this);
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
  start_service_ = n_.advertiseService("/start", &HLController::callbackStart_, this);
  stop_service_ = n_.advertiseService("/shutdown", &HLController::callbackShutdown_, this);
  set_upper_limit_service_ = n_.advertiseService("/set_upper_limit", &HLController::callbackSetUpperLimit_, this);
  set_lower_limit_service_ = n_.advertiseService("/set_lower_limit", &HLController::callbackSetLowerLimit_, this);
  reset_upper_limit_service_ = n_.advertiseService("/reset_upper_limit", &HLController::callbackResetUpperLimit_, this);
  reset_lower_limit_service_ = n_.advertiseService("/reset_lower_limit", &HLController::callbackResetLowerLimit_, this);
  enable_upper_limit_service_ =
      n_.advertiseService("/enable_upper_limit", &HLController::callbackEnableUpperLimit_, this);
  enable_lower_limit_service_ =
      n_.advertiseService("/enable_lower_limit", &HLController::callbackEnableLowerLimit_, this);

  ros::service::waitForService("/controller_manager/switch_controller");
  ros::service::waitForService("/start_motor");
  ros::service::waitForService("/stop_motor");
  switch_ctrl_srv_ = n_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/"
                                                                                 "switch_controller");
  start_motor_control_srv_ = n_.serviceClient<std_srvs::Empty>("/start_motor");
  stop_motor_control_srv_ = n_.serviceClient<std_srvs::Empty>("/stop_motor");
}

void HLController::retrieveParameters_()
{
  ROS_DEBUG_NAMED("HLController", "retrieveParameters");
  ros::param::get("~sampling_frequency", sampling_freq_);
  ros::param::get("~direction", direction_);
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);
  ros::param::get("/joint_limits/joint1/min_position", lower_limit_.data);
  ros::param::get("/joint_limits/joint1/max_position", upper_limit_.data);
  ros::param::get("/hl_controller_node/close_loop_control", close_loop_control_);
}

void HLController::initializeStateMachine_()
{
  /* State definition */
  state_uninitialize_ = new State<HLController>(this, "UNINITIALIZED");
  state_uninitialize_->registerEnterFcn(&HLController::uninitializedEnter_);

  state_running_ = new State<HLController>(this, "RUNNING");
  state_running_->registerEnterFcn(&HLController::runningEnter_);
  state_running_->registerUpdateFcn(&HLController::runningUpdate_);

  state_shutting_down_ = new State<HLController>(this, "SHUTTING DOWN");
  state_shutting_down_->registerEnterFcn(&HLController::shuttingDownEnter_);

  state_error_processing_ = new State<HLController>(this, "ERROR PROCESSING");
  state_error_processing_->registerEnterFcn(&HLController::errorProcessingEnter_);

  state_finalized_ = new State<HLController>(this, "FINALIZED");
  state_finalized_->registerEnterFcn(&HLController::finalizedEnter_);

  /* Transitions definition */
  tr_error_raised_ = new Transition<HLController>(this, state_error_processing_);
  tr_error_raised_->registerConditionFcn(&HLController::trErrorRaised_);
  tr_error_raised_->addInitialState(state_running_);
  tr_error_raised_->addInitialState(state_shutting_down_);

  tr_to_running_ = new Transition<HLController>(this, state_running_);
  tr_to_running_->registerConditionFcn(&HLController::trToRunning_);
  tr_to_running_->addInitialState(state_uninitialize_);

  tr_test_ = new Transition<HLController>(this, state_running_);
  tr_test_->registerConditionFcn(&HLController::trTest_);
  tr_test_->addInitialState(state_shutting_down_);

  tr_to_shutting_down_ = new Transition<HLController>(this, state_shutting_down_);
  tr_to_shutting_down_->registerConditionFcn(&HLController::trToShuttingDown_);
  tr_to_shutting_down_->addInitialState(state_uninitialize_);
  tr_to_shutting_down_->addInitialState(state_running_);

  tr_error_success_ = new Transition<HLController>(this, state_uninitialize_);
  tr_error_success_->registerConditionFcn(&HLController::trErrorSuccess_);
  tr_error_success_->addInitialState(state_error_processing_);

  tr_error_failure_ = new Transition<HLController>(this, state_finalized_);
  tr_error_failure_->registerConditionFcn(&HLController::trErrorFailure_);
  tr_error_failure_->addInitialState(state_error_processing_);

  /* Engine definition */
  engine_ = new Engine<HLController>(this);
  engine_->registerState(state_uninitialize_);
  engine_->registerState(state_running_);
  engine_->registerState(state_shutting_down_);
  engine_->registerState(state_error_processing_);
  engine_->registerState(state_finalized_);

  engine_->registerTransition(tr_error_raised_);
  engine_->registerTransition(tr_to_running_);
  engine_->registerTransition(tr_to_shutting_down_);
  engine_->registerTransition(tr_error_success_);
  engine_->registerTransition(tr_error_failure_);
  engine_->registerTransition(tr_test_);

  engine_->setCurrentState(state_uninitialize_);
}

/****** FSM State function definition ******/
void HLController::uninitializedEnter_()
{
  ROS_WARN_NAMED("HLController", "uninitializedEnter_");
  // if (stopMotor_() != OK)
  // {
  //   status_ = ERROR;
  // }
}

void HLController::runningEnter_()
{
  ROS_WARN_NAMED("HLController", "runningEnter_");

  if (startMotor_() != OK)
  {
    status_ = ERROR;
  }
}

void HLController::runningUpdate_()
{
  ROS_WARN_NAMED("HLController", "runningUpdate_");

  double input_velocity_cmd;

  /* Prioritize the input velocity command. GUI is higher priority so we take joypad command only
   * if no GUI command is received */
  if (gui_cmd_.data != 0.0)
  {
    input_velocity_cmd = gui_cmd_.data;
    ROS_INFO_NAMED("HLController", "velocity command received from gui : %f", input_velocity_cmd);
  }
  else if (joy_cmd_.data != 0.0)
  {
    input_velocity_cmd = joy_cmd_.data;
    ROS_INFO_NAMED("HLController", "velocity command received from joy : %f", input_velocity_cmd);
  }
  else if (tb_cmd_.data != 0.0)
  {
    input_velocity_cmd = tb_cmd_.data;
    ROS_INFO_NAMED("HLController", "velocity command received from twin button : %f", input_velocity_cmd);
  }
  else
  {
    input_velocity_cmd = 0.0;
    ROS_INFO_NAMED("HLController", "no velocity command received !");
  }

  /* Take velocity set from gui if available */
  if (speed_setpoint_ != speed_setpoint_)
  {
    /* setpoint is NaN : do not use it */
  }
  else if (input_velocity_cmd != 0.0)
  {
    ROS_WARN_NAMED("HLController", "Velocity setpoint receive from gui : %f", speed_setpoint_);
    if (input_velocity_cmd < 0.0)
    {
      input_velocity_cmd = -speed_setpoint_;
    }
    else
    {
      input_velocity_cmd = speed_setpoint_;
    }
  }

  adaptVelocityNearLimits_(input_velocity_cmd, reduced_speed_divisor_);
  input_velocity_cmd *= direction_;

  ROS_INFO_NAMED("HLController", "input_velocity_cmd : %f", input_velocity_cmd);

  if (close_loop_control_)
  {
    cmd_.data = joint_angles_.data;
  }

  cmd_.data = cmd_.data + (input_velocity_cmd * sampling_period_);
  handleLimits_(cmd_.data);

  ROS_INFO_NAMED("HLController", "output position command : %f", cmd_.data);

  // cmd
  cmd_pub_.publish(cmd_);
  upper_limit_pub_.publish(upper_limit_);
  lower_limit_pub_.publish(lower_limit_);
  joint_angle_pub_.publish(joint_angles_);
}

void HLController::shuttingDownEnter_()
{
  ROS_WARN_NAMED("HLController", "shuttingDownEnter_");

  if (stopMotor_() != OK)
  {
    status_ = ERROR;
  }
}

void HLController::errorProcessingEnter_()
{
  ROS_WARN_NAMED("HLController", "errorProcessingEnter_");

  /* TODO : For now, all error lead to Finalized state. */
}

void HLController::finalizedEnter_()
{
  ROS_WARN_NAMED("HLController", "finalizedEnter_");

  ros::shutdown();
}

/******** FSM Transition definition ********/
bool HLController::trErrorRaised_()
{
  // return (status_ == ERROR);
  return false;
}

bool HLController::trToRunning_()
{
  return (input_event_requested_ == FsmInputEvent::Start);
}

bool HLController::trTest_()
{
  return (input_event_requested_ == FsmInputEvent::Shutdown);
}

bool HLController::trToShuttingDown_()
{
  return (input_event_requested_ == FsmInputEvent::Shutdown);
}

bool HLController::trErrorSuccess_()
{
  return (status_ == OK);
}

bool HLController::trErrorFailure_()
{
  return (status_ == ERROR);
}
/*******************************************/

void HLController::init_()
{
  upper_limit_.data = NAN;
  lower_limit_.data = NAN;
  speed_setpoint_ = NAN;
}

HLController::status_t HLController::startMotor_()
{
  ROS_WARN_NAMED("HLController", "startMotor_");

  controller_manager_msgs::SwitchController msgController;
  std_srvs::Empty msgMotor;
  msgController.request.start_controllers = { { "/at1x/controller/position/joint1" } };
  msgController.request.stop_controllers = {};
  msgController.request.strictness = 1;
  if (!switch_ctrl_srv_.call(msgController))
  {
    ROS_ERROR_NAMED("HLController", "Problem occured during controller switch to start.");
    /* TODO handle error here */
  }
  else
  {
    if (!start_motor_control_srv_.call(msgMotor))
    {
      ROS_ERROR_NAMED("HLController", "Problem occured during motor start.");
      /* TODO handle error here */
    }
    else
    {
      return OK;
    }
  }
  return ERROR;
}

HLController::status_t HLController::stopMotor_()
{
  ROS_WARN_NAMED("HLController", "stopMotor_");

  controller_manager_msgs::SwitchController msgController;
  std_srvs::Empty msgMotor;
  msgController.request.start_controllers = {};
  msgController.request.stop_controllers = { { "/at1x/controller/position/joint1" } };
  if (!switch_ctrl_srv_.call(msgController))
  {
    ROS_ERROR_NAMED("HLController", "Problem occured during controller switch to stop.");
    /* TODO handle error here */
  }
  else
  {
    if (!stop_motor_control_srv_.call(msgMotor))
    {
      ROS_ERROR_NAMED("HLController", "Problem occured during motor stop.");
      /* TODO handle error here */
    }
    else
    {
      return OK;
    }
  }
  return ERROR;
}

void HLController::handleLimits_(double& cmd)
{
  if (cmd > upper_limit_.data)
  {
    cmd = upper_limit_.data;
  }
  if (cmd < lower_limit_.data)
  {
    cmd = lower_limit_.data;
  }
}

void HLController::adaptVelocityNearLimits_(double& cmd, const float divisor)
{
  if (upper_limit_.data != NAN && enable_upper_limit)
  {
    /* If upper limit is set */
    if (joint_angles_.data > upper_limit_.data)
    {
      /* If we have exceed the upper limit, we can only go in reverse direction */
      if (cmd > 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("HLController", "Upper limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd > 0.0) && (upper_limit_.data - joint_angles_.data < (abs(cmd) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("HLController", "Close to upper limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((upper_limit_.data - joint_angles_.data) / (2.0 * abs(cmd) / divisor));
    }
  }

  if (lower_limit_.data != NAN && enable_lower_limit)
  {
    /* If lower limit is set */
    if (joint_angles_.data < lower_limit_.data)
    {
      /* If we have exceed the lower limit, we can only go in direct direction */
      if (cmd < 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("HLController", "Lower limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd < 0.0) && (joint_angles_.data - lower_limit_.data < (abs(cmd) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("HLController", "Close to lower limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((joint_angles_.data - lower_limit_.data) / (2.0 * abs(cmd) / divisor));
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
  ROS_DEBUG_NAMED("HLController", "callbackJointStates_");
  joint_angles_.data = direction_ * msg->position[0];
  ROS_DEBUG_NAMED("HLController", "joint_angles_.data : %f", joint_angles_.data);
}

void HLController::callbackJoyCmd_(const std_msgs::Float64Ptr& msg)
{
  joy_cmd_.data = msg->data;
}

void HLController::callbackGuiCmd_(const std_msgs::Float64Ptr& msg)
{
  gui_cmd_.data = msg->data;
}

void HLController::callbackTbCmd_(const std_msgs::Float64Ptr& msg)
{
  tb_cmd_.data = msg->data;
}

void HLController::callbackSpeedSetpoint_(const std_msgs::Float64Ptr& msg)
{
  speed_setpoint_ = msg->data;
}

bool HLController::callbackStart_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackStart_");
  input_event_requested_ = FsmInputEvent::Start;
  return true;
}

bool HLController::callbackShutdown_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_NAMED("HLController", "callbackShutdown_");
  input_event_requested_ = FsmInputEvent::Shutdown;
  return true;
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
