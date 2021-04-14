//============================================================================
// Name        : armms_power_button_led.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_rpi/armms_power_button_led.h"

namespace armms_rpi
{
ArmmsPowerButtonLed::ArmmsPowerButtonLed(const ros::NodeHandle& nh) : nh_(nh)
{
  retrieveParameters_();
  initializeServices_();

  led_blink_speed_ = 0;
  led_blink_counter_ = 0;
  led_blink_state_ = true;
  long_press_detected_ = false;
  press_counter_ = 0;
  /* Led is initialized in red color */
  setLedColor(255, 0, 0, 0);

  pinMode(power_btn_pin_, INPUT);
  pinMode(red_led_pin_, OUTPUT);
  pinMode(green_led_pin_, OUTPUT);
  pinMode(blue_led_pin_, OUTPUT);
  pullUpDnControl(power_btn_pin_, PUD_OFF);

  if (softPwmCreate(red_led_pin_, red_led_state_, 100) != 0)
  {
    ROS_ERROR_NAMED("ArmmsPowerButtonLed", "red_led_pin_ softPwmCreate error !");
    return;
  }

  if (softPwmCreate(green_led_pin_, green_led_state_, 100) != 0)
  {
    ROS_ERROR_NAMED("ArmmsPowerButtonLed", "green_led_pin_ softPwmCreate error !");
    return;
  }

  if (softPwmCreate(blue_led_pin_, blue_led_state_, 100) != 0)
  {
    ROS_ERROR_NAMED("ArmmsPowerButtonLed", "blue_led_pin_ softPwmCreate error !");
    return;
  }
}

ArmmsPowerButtonLed::~ArmmsPowerButtonLed()
{
  /* Stop PWM and setup led to red */
  softPwmStop(red_led_pin_);
  softPwmStop(green_led_pin_);
  softPwmStop(blue_led_pin_);
  digitalWrite(red_led_pin_, 0);
  digitalWrite(green_led_pin_, 1);
  digitalWrite(blue_led_pin_, 1);
}

void ArmmsPowerButtonLed::update()
{
  updateLed_();
  processPowerButtonInput_();
}

void ArmmsPowerButtonLed::setLedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed)
{
  led_blink_speed_ = blink_speed;
  /* Warning : due to high side led wiring, output states are inverted
   * so that 0 mean 100% while 100 mean 0% .*/
  red_led_state_ = 100 - scaleColorPwm_(r);
  green_led_state_ = 100 - scaleColorPwm_(g);
  blue_led_state_ = 100 - scaleColorPwm_(b);
  /* Reset Blink counter when led color is set */
  led_blink_counter_ = 0;
  /* Force led state. This allow to set led state without calling updateLed_ method */
  softPwmWrite(red_led_pin_, red_led_state_);
  softPwmWrite(green_led_pin_, green_led_state_);
  softPwmWrite(blue_led_pin_, blue_led_state_);
}

void ArmmsPowerButtonLed::updateLed_()
{
  if (led_blink_speed_ == 0)
  {
    led_blink_state_ = true;
  }
  else
  {
    if (led_blink_counter_ > led_blink_speed_)
    {
      led_blink_counter_ = 0;
      led_blink_state_ = !led_blink_state_;
    }
  }
  led_blink_counter_++;
  if (led_blink_state_)
  {
    /* Set led to user defined value */
    softPwmWrite(red_led_pin_, red_led_state_);
    softPwmWrite(green_led_pin_, green_led_state_);
    softPwmWrite(blue_led_pin_, blue_led_state_);
  }
  else
  {
    /* Set led to off */
    softPwmWrite(red_led_pin_, 100);
    softPwmWrite(green_led_pin_, 100);
    softPwmWrite(blue_led_pin_, 100);
  }
}

void ArmmsPowerButtonLed::processPowerButtonInput_()
{
  /* High side commutation */
  power_btn_state_ = digitalRead(power_btn_pin_);

  if (power_btn_state_ == 1)
  {
    if (power_btn_prev_state_ == 0)
    {
      /* Rising edge of button */
      press_time_ = ros::Time::now();
    }
    /* Raise event if button is maintained during long time */
    if (ros::Duration(ros::Time::now() - press_time_) > long_press_duration_)
    {
      /* Long press detected */
      button_action_ = BTN_LONG_PRESS;
      press_time_ = ros::Time::now();
      long_press_detected_ = true;
    }
  }
  else
  {
    /* Falling edge detected */
    if (power_btn_prev_state_ == 1)
    {
      if(long_press_detected_ != true)
      {
        /* short press detected */
        press_counter_++;
        release_time_ = ros::Time::now();
      }
      else
      {
        /* If falling edge of a long press : do not take into account this event */
        long_press_detected_ = false;
      }
    }
    if (press_counter_ > 0)
    {
      /* After a specified inactivity delay, raise button event according to number of press detected */
      if (ros::Duration(ros::Time::now() - release_time_) > inactivity_duration_)
      {
        if (press_counter_ == 1)
        {
          button_action_ = BTN_SHORT_PRESS;
        }
        else if (press_counter_ == 2)
        {
          button_action_ = BTN_SHORT_DOUBLE_PRESS;
        }
        else
        {
          button_action_ = BTN_NONE;
          press_counter_ = 0;
        }
      }
    }
  }

  /* Only send service request if an event is detected */
  if (button_action_ != BTN_NONE)
  {
    press_counter_ = 0;
    if (button_event_srv_.exists())
    {
      armms_msgs::ButtonEvent msgButton;
      msgButton.request.button_event = button_action_;
      if (!button_event_srv_.call(msgButton))
      {
        ROS_ERROR_NAMED("ArmmsPowerButtonLed", "Problem when raising button event");
      }
    }
  }

  power_btn_prev_state_ = power_btn_state_;
  button_action_ = BTN_NONE;
}

void ArmmsPowerButtonLed::retrieveParameters_()
{
  ros::param::get("~power_button_pin", power_btn_pin_);
  ros::param::get("~red_led_pin", red_led_pin_);
  ros::param::get("~green_led_pin", green_led_pin_);
  ros::param::get("~blue_led_pin", blue_led_pin_);
  double duration;
  ros::param::get("~long_press_duration", duration);
  long_press_duration_ = ros::Duration(duration);
  ros::param::get("~inactivity_duration", duration);
  inactivity_duration_ = ros::Duration(duration);
}

void ArmmsPowerButtonLed::initializeServices_()
{
  set_rgb_led_service_ = nh_.advertiseService("/armms_rpi/set_rgb_led", &ArmmsPowerButtonLed::callbackSetRGBLed_, this);
  button_event_srv_ = nh_.serviceClient<armms_msgs::ButtonEvent>("/armms_rpi/power_button_event");
}

bool ArmmsPowerButtonLed::callbackSetRGBLed_(armms_msgs::SetLedColor::Request& req,
                                             armms_msgs::SetLedColor::Response& res)
{
  setLedColor(req.r, req.g, req.b, req.blink_speed);
  return true;
}

int ArmmsPowerButtonLed::scaleColorPwm_(uint8_t color)
{
  int result = ((int)color * 100) / 255;
  if (result > 100)
  {
    result = 100;
  }
  if (result < 0)
  {
    result = 0;
  }
  return result;
}
}  // namespace armms_rpi

using namespace armms_rpi;
