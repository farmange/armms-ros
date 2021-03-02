//============================================================================
// Name        : armms_at1x_control.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_control/armms_at1x_control.h"

namespace armms_control
{
ArmmsAT1XControl::ArmmsAT1XControl() : enable_upper_limit_(true), enable_lower_limit_(true)
{
  sampling_frequency_ = 0;
  init_();
  retrieveParameters_();
  initializeServices_();
  initializeSubscribers_();
  initializePublishers_();
  initializeStateMachine_();

  if (sampling_frequency_ > 0)
  {
    // TODO unused sampling period
    sampling_period_ = 1.0 / sampling_frequency_;
  }
  else
  {
    ROS_ERROR("Sampling frequency could not be lower or equal to zero : %d", sampling_frequency_);
    ros::shutdown();
    return;
  }

  ros::Rate loop_rate = ros::Rate(sampling_frequency_);

  /* Define a speed divisor, for slow down phases. The value 15.0 could be interpreted as the position in degree where
   * slow down happened. Here, we will start to decrease the speed as soon as we will be under 15 degree from the
   * defined limit.
   */
  reduced_speed_divisor_ = joint_max_speed_ / 15.0;

  input_event_requested_ = FsmInputEvent::None;

  // TODO remove unused timer etc
  // ros::Duration update_time = ros::Duration(1.0 / sampling_frequency_);
  // non_realtime_loop_ = nh_.createTimer(update_time, &ArmmsAT1XControl::update_, this);
  // ros::spin();

  while (ros::ok())
  {
    ros::spinOnce();
    /**** FSM ****/
    ROS_DEBUG_NAMED("ArmmsAT1XControl", "Current state is '%s' and input event is '%s'",
                    engine_->getCurrentState()->getName().c_str(), input_event_requested_.toString().c_str());
    engine_->process();
    input_event_requested_ = FsmInputEvent::None;
    /*************/

    loop_rate.sleep();
  }
}

void ArmmsAT1XControl::initializeServices_()
{
  ROS_DEBUG("initializeServices_");

  ros::service::waitForService("/armms_rpi/set_rgb_led");
  ros::service::waitForService("/armms_rpi/set_motor_power");
  set_led_color_srv_ = nh_.serviceClient<armms_msgs::SetLedColor>("/armms_rpi/set_rgb_led");
  set_motor_power_srv_ = nh_.serviceClient<armms_msgs::SetMotorPower>("/armms_rpi/set_motor_power");

  pwr_btn_ev_service_ =
      nh_.advertiseService("/armms_rpi/power_button_event", &ArmmsAT1XControl::callbackPowerButtonEvent_, this);
}

void ArmmsAT1XControl::initializeSubscribers_()
{
  ROS_DEBUG("initializeSubscribers_");
  // ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/user_button_down");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/user_button_up");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/switch_limit");
  ros::topic::waitForMessage<std_msgs::Bool>("/armms_rpi/motor_power");

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &ArmmsAT1XControl::callbackJointStates_, this);
  user_btn_down_sub_ = nh_.subscribe("/armms_rpi/user_button_down", 1, &ArmmsAT1XControl::callbackUserBtnDown_, this);
  user_btn_up_sub_ = nh_.subscribe("/armms_rpi/user_button_up", 1, &ArmmsAT1XControl::callbackUserBtnUp_, this);
  switch_limit_sub_ = nh_.subscribe("/armms_rpi/switch_limit", 1, &ArmmsAT1XControl::callbackSwitchLimit_, this);
  motor_power_sub_ = nh_.subscribe("/armms_rpi/motor_power", 1, &ArmmsAT1XControl::callbackMotorPower_, this);
}

void ArmmsAT1XControl::initializePublishers_()
{
  ROS_DEBUG("initializePublishers_");
  position_command_pub_ = nh_.advertise<std_msgs::Float64>("/at1x/controller/position/joint1/command", 1);
  upper_limit_pub_ = nh_.advertise<std_msgs::Float64>("/upper_limit", 1);
  lower_limit_pub_ = nh_.advertise<std_msgs::Float64>("/lower_limit", 1);
}

void ArmmsAT1XControl::retrieveParameters_()
{
  ROS_DEBUG("retrieveParameters");
  ros::param::get("~sampling_frequency", sampling_frequency_);
  ros::param::get("/joint_limits/joint1/max_velocity", joint_max_speed_);
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

  state_error_processing_ = new State<ArmmsAT1XControl>(this, "ERROR PROCESSING");
  state_error_processing_->registerEnterFcn(&ArmmsAT1XControl::errorProcessingEnter_);
  state_error_processing_->registerUpdateFcn(&ArmmsAT1XControl::errorProcessingUpdate_);

  state_finalized_ = new State<ArmmsAT1XControl>(this, "FINALIZED");
  state_finalized_->registerEnterFcn(&ArmmsAT1XControl::finalizedEnter_);

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

  tr_finalize_ = new Transition<ArmmsAT1XControl>(this, state_finalized_);
  tr_finalize_->registerConditionFcn(&ArmmsAT1XControl::trFinalize_);
  tr_finalize_->addInitialState(state_stopped_);
  tr_finalize_->addInitialState(state_position_control_);
  tr_finalize_->addInitialState(state_error_processing_);

  tr_error_success_ = new Transition<ArmmsAT1XControl>(this, state_stopped_);
  tr_error_success_->registerConditionFcn(&ArmmsAT1XControl::trErrorSuccess_);
  tr_error_success_->addInitialState(state_error_processing_);

  tr_error_critical_ = new Transition<ArmmsAT1XControl>(this, state_finalized_);
  tr_error_critical_->registerConditionFcn(&ArmmsAT1XControl::trErrorCritical_);
  tr_error_critical_->addInitialState(state_error_processing_);

  tr_stop_ = new Transition<ArmmsAT1XControl>(this, state_stopped_);
  tr_stop_->registerConditionFcn(&ArmmsAT1XControl::trStop_);
  tr_stop_->addInitialState(state_position_control_);

  /* Engine definition */
  engine_ = new Engine<ArmmsAT1XControl>(this);
  engine_->registerState(state_uninitialized_);
  engine_->registerState(state_stopped_);
  engine_->registerState(state_position_control_);
  engine_->registerState(state_error_processing_);
  engine_->registerState(state_finalized_);

  engine_->registerTransition(tr_initialized_);
  engine_->registerTransition(tr_error_raised_);
  engine_->registerTransition(tr_to_position_control_);
  engine_->registerTransition(tr_finalize_);
  engine_->registerTransition(tr_error_success_);
  engine_->registerTransition(tr_error_critical_);
  engine_->registerTransition(tr_stop_);

  engine_->setCurrentState(state_uninitialized_);
}

/****** FSM State function definition ******/
void ArmmsAT1XControl::uninitializedEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "uninitializedEnter_");
  setLedColor_(255, 0, 0, 1);  // red fast blink
}

void ArmmsAT1XControl::stoppedEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "stoppedEnter_");
  if (stopMotor_() != OK)
  {
    status_ = ERROR;
  }
  // TODO clarify stop motor before
  setLedColor_(255, 0, 0, 0);  // red
}

void ArmmsAT1XControl::positionControlEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "positionControlEnter_");
  refresh_joint_state_ = true;
  if (startMotor_() != OK)
  {
    status_ = ERROR;
  }
  setLedColor_(0, 255, 0, 0);  // green
}

void ArmmsAT1XControl::positionControlUpdate_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "positionControlUpdate_");

  double input_velocity_cmd;
  ros::Duration test = ros::Duration(ros::Time::now() - joint_position_time_);
  ROS_WARN_STREAM_NAMED("ArmmsAT1XControl", "test : " << test);

  if (test > ros::Duration(0.1))
  {
    refresh_joint_state_ = true;
  }
  // /* Ensure joint_angle_ is up to date and refresh local copy (cmd_) */
  if (refresh_joint_state_)
  {
    cmd_.data = joint_position_;
    refresh_joint_state_ = false;
    return;
  }

  if (user_btn_up_ && user_btn_down_)
  {
    /* do nothing */
    input_velocity_cmd = 0.0;
    ROS_INFO_NAMED("ArmmsAT1XControl", "Conflicting up and down button pressed at the same time !");
  }
  else if (user_btn_up_)
  {
    input_velocity_cmd = +joint_max_speed_;
    ROS_INFO_NAMED("ArmmsAT1XControl", "velocity command received from up button : %f", input_velocity_cmd);
  }
  else if (user_btn_down_)
  {
    input_velocity_cmd = -joint_max_speed_;
    ROS_INFO_NAMED("ArmmsAT1XControl", "velocity command received from down button : %f", input_velocity_cmd);
  }
  else
  {
    input_velocity_cmd = 0.0;
    ROS_INFO_NAMED("ArmmsAT1XControl", "no velocity command received !");
  }

  adaptVelocityNearLimits_(input_velocity_cmd, reduced_speed_divisor_);

  ROS_INFO_NAMED("ArmmsAT1XControl", "input_velocity_cmd : %f", input_velocity_cmd);

  /* Speed integration to retrieve position command */
  cmd_.data = cmd_.data + (input_velocity_cmd * sampling_period_);
  handleLimits_(cmd_.data);

  ROS_INFO_NAMED("ArmmsAT1XControl", "output position command : %f", cmd_.data);

  position_command_pub_.publish(cmd_);
  upper_limit_pub_.publish(upper_limit_);
  lower_limit_pub_.publish(lower_limit_);
  // joint_angle_pub_.publish(joint_angles_);
}

void ArmmsAT1XControl::errorProcessingEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "errorProcessingEnter_");
  setLedColor_(255, 0, 0, 10);  // red + bling 100ms
  if (stopMotor_() != OK)
  {
    status_ = ERROR;
  }
}

void ArmmsAT1XControl::errorProcessingUpdate_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "errorProcessingUpdate_");
}

void ArmmsAT1XControl::finalizedEnter_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "finalizedEnter_");
  setLedColor_(0, 0, 0, 0);  // power off led
}

/******** FSM Transition definition ********/
bool ArmmsAT1XControl::trInitialized_()
{
  return true;
}

bool ArmmsAT1XControl::trErrorRaised_()
{
  // return (status_ == ERROR);
  // TODO handle errors !
  return false;
}

bool ArmmsAT1XControl::trToPositionControl_()
{
  return ((input_event_requested_ == FsmInputEvent::ButtonShortPress));
}

bool ArmmsAT1XControl::trFinalize_()
{
  return (input_event_requested_ == FsmInputEvent::ButtonLongPress);
}

bool ArmmsAT1XControl::trErrorSuccess_()
{
  return (status_ == OK);
}

bool ArmmsAT1XControl::trErrorCritical_()
{
  return (status_ == ERROR);
}

bool ArmmsAT1XControl::trStop_()
{
  return (input_event_requested_ == FsmInputEvent::ButtonShortPress);
}

/*******************************************/

void ArmmsAT1XControl::update_(const ros::TimerEvent&)
{
}

void ArmmsAT1XControl::callbackJointStates_(const sensor_msgs::JointStatePtr& msg)
{
  joint_position_time_ = ros::Time(msg->header.stamp);
  joint_position_ = msg->position[0];
}

void ArmmsAT1XControl::callbackUserBtnDown_(const std_msgs::BoolPtr& msg)
{
  /* Detect rising edge of the button to reset local joint position */
  if (!user_btn_down_ && msg->data)
  {
    refresh_joint_state_ = true;
  }
  user_btn_down_ = msg->data;
}

void ArmmsAT1XControl::callbackUserBtnUp_(const std_msgs::BoolPtr& msg)
{
  /* Detect rising edge of the button to reset local joint position */
  if (!user_btn_up_ && msg->data)
  {
    refresh_joint_state_ = true;
  }
  user_btn_up_ = msg->data;
}

void ArmmsAT1XControl::callbackSwitchLimit_(const std_msgs::BoolPtr& msg)
{
  switch_limit_ = msg->data;
}

void ArmmsAT1XControl::callbackMotorPower_(const std_msgs::BoolPtr& msg)
{
  motor_power_ = msg->data;
}

bool ArmmsAT1XControl::callbackPowerButtonEvent_(armms_msgs::ButtonEvent::Request& req,
                                                 armms_msgs::ButtonEvent::Response& res)
{
  ROS_INFO_NAMED("ArmmsAT1XControl", "callbackPowerButtonEvent_");
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
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Button event not supported (%d)", req.button_event);
    return false;
  }
  return true;
}

/**********************************************************/

void ArmmsAT1XControl::init_()
{
  upper_limit_.data = NAN;
  lower_limit_.data = NAN;
  speed_setpoint_ = NAN;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::startMotor_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "startMotor_");
  armms_msgs::SetMotorPower msgPower;
  msgPower.request.power_state = 1;
  if (!set_motor_power_srv_.call(msgPower))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem occured during power enable.");
    /* TODO handle error here */
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::stopMotor_()
{
  ROS_WARN_NAMED("ArmmsAT1XControl", "stopMotor_");
  armms_msgs::SetMotorPower msgPower;
  msgPower.request.power_state = 0;
  if (!set_motor_power_srv_.call(msgPower))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem occured during power disable.");
    /* TODO handle error here */
    return ERROR;
  }
  return OK;
}

ArmmsAT1XControl::status_t ArmmsAT1XControl::setLedColor_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed)
{
  ROS_DEBUG_NAMED("ArmmsAT1XControl", "setLedColor_");

  armms_msgs::SetLedColor msgColor;
  msgColor.request.r = r;
  msgColor.request.g = g;
  msgColor.request.b = b;
  msgColor.request.blink_speed = blink_speed;

  if (!set_led_color_srv_.call(msgColor))
  {
    ROS_ERROR_NAMED("ArmmsAT1XControl", "Problem when calling set led color service");
    /* TODO handle error here */
    return ERROR;
  }
  return OK;
}

void ArmmsAT1XControl::handleLimits_(double& cmd)
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

void ArmmsAT1XControl::adaptVelocityNearLimits_(double& cmd, const float divisor)
{
  if (upper_limit_.data != NAN && enable_upper_limit)
  {
    /* If upper limit is set */
    if (joint_position_ > upper_limit_.data)
    {
      /* If we have exceed the upper limit, we can only go in reverse direction */
      if (cmd > 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("ArmmsAT1XControl", "Upper limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd > 0.0) && (upper_limit_.data - joint_position_ < (abs(cmd) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("ArmmsAT1XControl", "Close to upper limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((upper_limit_.data - joint_position_) / (2.0 * abs(cmd) / divisor));
    }
  }

  if (lower_limit_.data != NAN && enable_lower_limit)
  {
    /* If lower limit is set */
    if (joint_position_ < lower_limit_.data)
    {
      /* If we have exceed the lower limit, we can only go in direct direction */
      if (cmd < 0.0)
      {
        ROS_DEBUG_STREAM_NAMED("ArmmsAT1XControl", "Lower limit overshooted. Cannot move in this direction !");
        cmd = 0.0;
      }
    }
    else if ((cmd < 0.0) && (joint_position_ - lower_limit_.data < (abs(cmd) / divisor)))
    {
      ROS_DEBUG_STREAM_NAMED("ArmmsAT1XControl", "Close to lower limit, automatic slow down ! ");
      /* In this code section, we exponentially decrease the speed as we get closer to the defined limit */
      cmd *= ((joint_position_ - lower_limit_.data) / (2.0 * abs(cmd) / divisor));
    }
  }
}

}  // namespace armms_control

using namespace armms_control;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_at1x_control_node");

  ros::NodeHandle nh;

  ArmmsAT1XControl at1x;

  ROS_INFO("shutdown node");
}