//============================================================================
// Name        : armms_power_button_led.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : TODO all doc of armms_rpi
//============================================================================

#ifndef ARMMS_RPI_POWER_BUTTON_LED_H
#define ARMMS_RPI_POWER_BUTTON_LED_H

#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "armms_msgs/SetLedColor.h"
#include "armms_msgs/ButtonEvent.h"

namespace armms_rpi
{
class ArmmsPowerButtonLed
{
public:
  ArmmsPowerButtonLed(const ros::NodeHandle& nh);
  ~ArmmsPowerButtonLed();
  void update();
  void setLedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer set_rgb_led_service_;
  ros::ServiceClient button_event_srv_;

  uint8_t led_blink_speed_;
  uint8_t led_blink_counter_;
  bool led_blink_state_;
  uint8_t red_led_state_;
  uint8_t green_led_state_;
  uint8_t blue_led_state_;
  int power_btn_state_;
  int power_btn_prev_state_;
  bool long_press_detected_;

  typedef enum button_action_e
  {
    BTN_NONE = 0,
    BTN_SHORT_PRESS,
    BTN_SHORT_DOUBLE_PRESS,
    BTN_LONG_PRESS
  } button_action_t;
  button_action_t button_action_;
  ros::Duration long_press_duration_;
  ros::Duration inactivity_duration_;
  ros::Time press_time_;
  ros::Time release_time_;
  int press_counter_;

  /* Input pins */
  int power_btn_pin_;

  /* Output pins */
  int red_led_pin_;
  int green_led_pin_;
  int blue_led_pin_;

  void retrieveParameters_();
  void initializeServices_();
  bool callbackSetRGBLed_(armms_msgs::SetLedColor::Request& req, armms_msgs::SetLedColor::Response& res);
  int scaleColorPwm_(uint8_t color);
  void updateLed_();
  void processPowerButtonInput_();
};

}  // namespace armms_rpi
#endif
