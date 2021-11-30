//============================================================================
// Name        : armms_at1x_control.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_control/armms_at1x_control.h"

namespace armms_control
{
ArmmsAT1XControl::ArmmsAT1XControl() : limit_handler_(joint_position_)
{
  loop_rate_ = 0;
  status_ = OK;
  retrieveParameters_();
  initializeServices_();
  initializeSubscribers_();
  initializePublishers_();
  initializeStateMachine_();

  if (loop_rate_ > 0)
  {
    sampling_period_ = 1.0 / loop_rate_;
  }
  else
  {
    ROS_ERROR("Sampling frequency could not be lower or equal to zero : %d", loop_rate_);
    ros::shutdown();
    return;
  }

  ros::Rate loop_rate = ros::Rate(loop_rate_);

  while (ros::ok())
  {
    ros::spinOnce();
    user_input_handler_.processUserInput();
    ROS_DEBUG_NAMED("ArmmsAT1XControl", "Current state is '%s' and input event is '%s'",
                    engine_->getCurrentState()->getName().c_str(),
                    user_input_handler_.getUserInput().toString().c_str());
    engine_->process();
    user_input_handler_.clearUserInput();

    loop_rate.sleep();
  }
  ROS_INFO_NAMED("ArmmsAT1XControl", "Shutdown...");
}

void ArmmsAT1XControl::initializeServices_()
{
  ros::service::waitForService("/armms_rpi/set_rgb_led");
  ros::service::waitForService("/armms_rpi/set_motor_power");
  ros::service::waitForService("/armms_rpi/shutdown_rpi");
  ros::service::waitForService("/armms_driver/reset_controller");
  set_led_color_srv_ = nh_.serviceClient<armms_msgs::SetLedColor>("/armms_rpi/set_rgb_led");
  set_motor_power_srv_ = nh_.serviceClient<armms_msgs::SetMotorPower>("/armms_rpi/set_motor_power");
  shutdown_srv_ = nh_.serviceClient<armms_msgs::SetInt>("/armms_rpi/shutdown_rpi");
  reset_controller_srv_ = nh_.serviceClient<std_srvs::Empty>("/armms_driver/reset_controller");
  user_intent_calib_srv_ = nh_.serviceClient<armms_msgs::UserIntentCalib>("/armms_user_intent/calibrate");
}

void ArmmsAT1XControl::initializeSubscribers_()
{
  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &ArmmsAT1XControl::callbackJointStates_, this);
}

void ArmmsAT1XControl::initializePublishers_()
{
  position_command_pub_ = nh_.advertise<std_msgs::Float64>("/at1x/controller/position/joint1/command", 1);
}

void ArmmsAT1XControl::retrieveParameters_()
{
  ros::param::get("~armms_control_rate", loop_rate_);
  ros::param::get("~slow_velocity_duration", slow_velocity_duration_);
  ros::param::get("~slow_velocity_setpoint", slow_velocity_setpoint_);
  ros::param::get("~acceleration_duration", acceleration_duration_);
  ros::param::get("~intent_ctrl_lo_limit", intent_ctrl_lo_limit_);
}

void ArmmsAT1XControl::initializeStateMachine_()
{
  /* State definition */
  state_uninitialized_ = new State<ArmmsAT1XControl>(this, "UNINITIALIZED");
  state_uninitialized_->registerEnterFcn(&ArmmsAT1XControl::uninitializedEnter_);

  state_stopped_ = new State<ArmmsAT1XControl>(this, "STOPPED");
  state_stopped_->registerEnterFcn(&ArmmsAT1XControl::stoppedEnter_);

  state_position_control_ = new State<ArmmsAT1XControl>(this, "POSITION CONTROL");
  state_position_control_->registerEnterFcn(&ArmmsAT1XControl::positionControlEnter_);
  state_position_control_->registerUpdateFcn(&ArmmsAT1XControl::positionControlUpdate_);

  state_intent_control_ = new State<ArmmsAT1XControl>(this, "INTENT CONTROL");
  state_intent_control_->registerEnterFcn(&ArmmsAT1XControl::intentControlEnter_);
  state_intent_control_->registerUpdateFcn(&ArmmsAT1XControl::intentControlUpdate_);

  state_intent_calibration_ = new State<ArmmsAT1XControl>(this, "INTENT CALIBRATION");
  state_intent_calibration_->registerEnterFcn(&ArmmsAT1XControl::intentCalibrationEnter_);

  state_error_processing_ = new State<ArmmsAT1XControl>(this, "ERROR PROCESSING");
  state_error_processing_->registerEnterFcn(&ArmmsAT1XControl::errorProcessingEnter_);
  state_error_processing_->registerUpdateFcn(&ArmmsAT1XControl::errorProcessingUpdate_);

  state_finalized_ = new State<ArmmsAT1XControl>(this, "FINALIZED");
  state_finalized_->registerEnterFcn(&ArmmsAT1XControl::finalizedEnter_);
  state_finalized_->registerUpdateFcn(&ArmmsAT1XControl::finalizedUpdate_);

  /* Transitions definition */
  tr_initialized_ = new Transition<ArmmsAT1XControl>(this, state_stopped_);
  tr_initialized_->registerConditionFcn(&ArmmsAT1XControl::trInitialized_);
  tr_initialized_->addInitialState(state_uninitialized_);

  tr_error_raised_ = new Transition<ArmmsAT1XControl>(this, state_error_processing_);
  tr_error_raised_->registerConditionFcn(&ArmmsAT1XControl::trErrorRaised_);
  tr_error_raised_->addInitialState(state_position_control_);
  tr_error_raised_->addInitialState(state_stopped_);

  tr_to_position_control_ = new Transition<ArmmsAT1XControl>(this, state_position_control_);
  tr_to_position_control_->registerConditionFcn(&ArmmsAT1XControl::trToPositionControl_);
  tr_to_position_control_->addInitialState(state_stopped_);

  tr_to_intent_control_ = new Transition<ArmmsAT1XControl>(this, state_intent_control_);
  tr_to_intent_control_->registerConditionFcn(&ArmmsAT1XControl::trToIntentControl_);
  tr_to_intent_control_->addInitialState(state_position_control_);

  tr_to_intent_calibration_ = new Transition<ArmmsAT1XControl>(this, state_intent_calibration_);
  tr_to_intent_calibration_->registerConditionFcn(&ArmmsAT1XControl::trToIntentCalibration_);
  tr_to_intent_calibration_->addInitialState(state_position_control_);

  tr_exit_intent_calibration_ = new Transition<ArmmsAT1XControl>(this, state_position_control_);
  tr_exit_intent_calibration_->registerConditionFcn(&ArmmsAT1XControl::trExitIntentCalibration_);
  tr_exit_intent_calibration_->addInitialState(state_intent_calibration_);

  tr_finalize_ = new Transition<ArmmsAT1XControl>(this, state_finalized_);
  tr_finalize_->registerConditionFcn(&ArmmsAT1XControl::trFinalize_);
  tr_finalize_->addInitialState(state_stopped_);
  tr_finalize_->addInitialState(state_position_control_);
  tr_finalize_->addInitialState(state_intent_control_);
  tr_finalize_->addInitialState(state_error_processing_);

  tr_error_success_ = new Transition<ArmmsAT1XControl>(this, state_stopped_);
  tr_error_success_->registerConditionFcn(&ArmmsAT1XControl::trErrorSuccess_);
  tr_error_success_->addInitialState(state_error_processing_);

  tr_error_critical_ = new Transition<ArmmsAT1XControl>(this, state_finalized_);
  tr_error_critical_->registerConditionFcn(&ArmmsAT1XControl::trErrorCritical_);
  tr_error_critical_->addInitialState(state_error_processing_);

  tr_stop_ = new Transition<ArmmsAT1XControl>(this, state_stopped_);
  tr_stop_->registerConditionFcn(&ArmmsAT1XControl::trStop_);
  tr_stop_->addInitialState(state_intent_control_);

  /* Engine definition */
  engine_ = new Engine<ArmmsAT1XControl>(this);
  engine_->registerState(state_uninitialized_);
  engine_->registerState(state_stopped_);
  engine_->registerState(state_position_control_);
  engine_->registerState(state_intent_control_);
  engine_->registerState(state_intent_calibration_);
  engine_->registerState(state_error_processing_);
  engine_->registerState(state_finalized_);

  engine_->registerTransition(tr_initialized_);
  engine_->registerTransition(tr_error_raised_);
  engine_->registerTransition(tr_to_position_control_);
  engine_->registerTransition(tr_to_intent_control_);
  engine_->registerTransition(tr_to_intent_calibration_);
  engine_->registerTransition(tr_exit_intent_calibration_);
  engine_->registerTransition(tr_finalize_);
  engine_->registerTransition(tr_error_success_);
  engine_->registerTransition(tr_error_critical_);
  engine_->registerTransition(tr_stop_);

  engine_->setCurrentState(state_uninitialized_);
}

/****** FSM State function definition ******/
void ArmmsAT1XControl::uninitializedEnter_()
{
  setLedColor_(255, 0, 0, 10);  // red blink
}

void ArmmsAT1XControl::stoppedEnter_()
{
  if (stopMotor_() != OK)
  {
    status_ = ERROR;
  }
  setLedColor_(0, 255, 0, 50);  // green + blink
}

void ArmmsAT1XControl::positionControlEnter_()
{
  refresh_joint_state_ = true;
  user_input_handler_.disableUserIntent();
  if (startMotor_() != OK)
  {
    status_ = ERROR;
  }
  setLedColor_(0, 255, 0, 0);  // green
}

void ArmmsAT1XControl::positionControlUpdate_()
{
  double input_velocity_cmd = user_input_handler_.getVelocityCommand();
  velocityControl_(input_velocity_cmd);
}

void ArmmsAT1XControl::intentCalibrationEnter_()
{
  setLedColor_(255, 0, 255, 0);  // blue + blink 50ms
  userIntentCalib_();
}

void ArmmsAT1XControl::intentControlEnter_()
{
  refresh_joint_state_ = true;
  user_input_handler_.enableUserIntent();
  setLedColor_(0, 0, 255, 0);  // blue
}

void ArmmsAT1XControl::intentControlUpdate_()
{
  bool isIntentCommand = false;
  double input_velocity_cmd = user_input_handler_.getVelocityCommand(isIntentCommand);
  /* PATCH : disable intent command when up command receive below the intent_ctrl_lo_limit.
   * This is to prevent intent control to go up when the arm of the user touch the armrest.
   * Note : the intent_ctrl_lo_limit parameter have to be correctly set according to user needs
   */
  if (isIntentCommand && (joint_position_ < intent_ctrl_lo_limit_) && (input_velocity_cmd > 0.0))
  {
    /* Do nothing and don't execute command */
    return;
  }
  velocityControl_(input_velocity_cmd);
}

void ArmmsAT1XControl::errorProcessingEnter_()
{
  setLedColor_(255, 0, 0, 10);  // red + blink 100ms
  ROS_WARN_NAMED("ArmmsAT1XControl", "Error detected (status : %d | switch_limit : %d)", status_,
                 user_input_handler_.getSwitchLimit());
  status_ = stopMotor_();
}

void ArmmsAT1XControl::errorProcessingUpdate_()
{
}

void ArmmsAT1XControl::finalizedEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "Control state machine enters in FINALIZED state...");
  status_ = stopMotor_();
  setLedColor_(255, 0, 0, 0);  // red
  shutdown_();
}

void ArmmsAT1XControl::finalizedUpdate_()
{
}

/******** FSM Transition definition ********/
bool ArmmsAT1XControl::trInitialized_()
{
  return true;
}

bool ArmmsAT1XControl::trErrorRaised_()
{
  /* Raise error if limit switch or error are present */
  // TODO FIX this to improve error handling and prevent reboot
  return (user_input_handler_.getSwitchLimit() || status_ != OK);
}

bool ArmmsAT1XControl::trToPositionControl_()
{
  return ((user_input_handler_.getUserInput() == FsmInputEvent::ButtonShortPress));
}

bool ArmmsAT1XControl::trToIntentControl_()
{
  return ((user_input_handler_.getUserInput() == FsmInputEvent::ButtonShortPress));
}

bool ArmmsAT1XControl::trToIntentCalibration_()
{
  return ((user_input_handler_.getUserInput() == FsmInputEvent::ButtonShortDoublePress));
}

bool ArmmsAT1XControl::trExitIntentCalibration_()
{
  return true;
}

bool ArmmsAT1XControl::trFinalize_()
{
  return (user_input_handler_.getUserInput() == FsmInputEvent::ButtonLongPress);
}

bool ArmmsAT1XControl::trErrorSuccess_()
{
  return (!user_input_handler_.getSwitchLimit() && status_ == OK);
}

bool ArmmsAT1XControl::trErrorCritical_()
{
  /* No critical error for now */
  return false;
}

bool ArmmsAT1XControl::trStop_()
{
  return (user_input_handler_.getUserInput() == FsmInputEvent::ButtonShortPress);
}

/*******************************************/

void ArmmsAT1XControl::callbackJointStates_(const sensor_msgs::JointStatePtr& msg)
{
  joint_position_time_ = ros::Time(msg->header.stamp);
  joint_position_ = msg->position[0];
  joint_torque_ = msg->effort[0];
}

/*******************************************/
void ArmmsAT1XControl::adaptAcceleration_(double& velocity_cmd)
{
  /* Detect rising edge */
  if (velocity_cmd != 0)
  {
    if (adapt_accel_rising_edge_detected == true)
    {
      adapt_accel_time_ = ros::Time::now();
    }
    adapt_accel_rising_edge_detected = false;
  }
  else
  {
    adapt_accel_rising_edge_detected = true;
    return;
  }

  /* If velocity command is lower than slow_velocity define in config file,
  then no acceleration adaptation is performed */
  if (abs(velocity_cmd) < slow_velocity_setpoint_)
  {
    return;
  }
  double step = 0.0;
  ros::Duration started_since_duration = ros::Duration(ros::Time::now() - adapt_accel_time_);
  ros::Duration long_cmd_duration = started_since_duration - ros::Duration(slow_velocity_duration_);

  /* If long command detected */
  if (long_cmd_duration > ros::Duration(0.0))
  {
    /* while rampup duration is not exceed */
    if (long_cmd_duration < ros::Duration(acceleration_duration_))
    {
      /* Compute the step to achieve to go from slow velocity to desired velocity */
      step = (abs(velocity_cmd) - slow_velocity_setpoint_);
      /* After slow_velocity_duration_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.toSec() / acceleration_duration_);
    }
    else
    {
      /* Do nothing (keep velocity_cmd as it is) */
      return;
    }
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (velocity_cmd > 0)
  {
    velocity_cmd = slow_velocity_setpoint_ + step;
  }
  else
  {
    velocity_cmd = -slow_velocity_setpoint_ - step;
  }
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::velocityControl_(const double& velocity_cmd)
{
  double velocity_cmd_limited = 0.0;
  /* If no fresh position was received since 100ms, stop publishing new commands */
  ros::Duration position_dt = ros::Duration(ros::Time::now() - joint_position_time_);
  if (position_dt > ros::Duration(0.1))
  {
    refresh_joint_state_ = true;
    ROS_WARN_NAMED("ArmmsAT1XControl", "No fresh position received :stall position control");
    return ERROR;
  }

  if (user_input_handler_.resetJointStateRequest())
  {
    refresh_joint_state_ = true;
  }

  /* Ensure joint_position_ is up to date and refresh local copy (cmd_) */
  if (refresh_joint_state_)
  {
    ROS_INFO_NAMED("ArmmsAT1XControl", "Refresh the joint position %f = %f : ", cmd_.data, joint_position_);
    cmd_.data = joint_position_;
    refresh_joint_state_ = false;
    return OK;  // TODO should we exit here ?
  }
  velocity_cmd_limited = velocity_cmd;
  adaptAcceleration_(velocity_cmd_limited);
  limit_handler_.adaptVelocityNearLimits(velocity_cmd_limited);

  ROS_DEBUG_NAMED("ArmmsAT1XControl", "velocity_cmd_limited : %f", velocity_cmd_limited);

  /* If positionning error is below 5.0 degres, do integration */
  if (abs(cmd_.data - joint_position_) < 5.0)
  {
    /* Speed integration to retrieve position command */
    cmd_.data = cmd_.data + (velocity_cmd_limited * sampling_period_);
    ROS_DEBUG_NAMED("ArmmsAT1XControl", "Computed position command is : %f", cmd_.data);
  }

  limit_handler_.saturatePosition(cmd_.data);
  limit_handler_.publishLimits();

  /* This detect falling edge of user command and force controller to reset and stop enforcelimit for example */
  if (velocity_cmd_limited == 0)
  {
    if (velocity_cmd_ != 0.0)
    {
      std_srvs::Empty dummy;
      cmd_.data = joint_position_;
      reset_controller_srv_.call(dummy);
    }
  }
  else
  {
    position_command_pub_.publish(cmd_);
  }
  velocity_cmd_ = velocity_cmd_limited;

  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::startMotor_()
{
  armms_msgs::SetMotorPower msgPower;
  msgPower.request.power_state = 1;
  if (!set_motor_power_srv_.call(msgPower))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem occured during power enable.");
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::stopMotor_()
{
  armms_msgs::SetMotorPower msgPower;
  msgPower.request.power_state = 0;
  if (!set_motor_power_srv_.call(msgPower))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem occured during power disable.");
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::userIntentCalib_()
{
  armms_msgs::UserIntentCalib msgCalib;
  msgCalib.request.value = 0.0;
  msgCalib.request.current_torque = true;
  if (!user_intent_calib_srv_.call(msgCalib))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem occured during user intent calibration.");
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::setLedColor_(const uint8_t& r, const uint8_t& g, const uint8_t& b,
                                                          const uint8_t& blink_speed)

{
  armms_msgs::SetLedColor msgColor;
  msgColor.request.r = r;
  msgColor.request.g = g;
  msgColor.request.b = b;
  msgColor.request.blink_speed = blink_speed;

  if (!set_led_color_srv_.call(msgColor))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem when calling set led color service");
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::shutdown_()
{
  armms_msgs::SetInt msgShutdown;
  msgShutdown.request.value = 1;  // shutdown

  if (!shutdown_srv_.call(msgShutdown))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem when calling shutdown service");
    return ERROR;
  }
  return OK;
}

}  // namespace armms_control

using namespace armms_control;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_at1x_control_node");

  ros::NodeHandle nh;

  ArmmsAT1XControl at1x;
}