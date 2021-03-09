//============================================================================
// Name        : armms_driver.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

// #include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
// #include <vector>
// #include <sstream>
// #include <string>

#include <boost/shared_ptr.hpp>
#include <thread>
#include <controller_manager/controller_manager.h>

#include "armms_driver/armms_api.h"
#include "armms_driver/armms_hardware_interface.h"
#include "armms_driver/armms_bag_recorder.h"

namespace armms_driver
{
class ArmmsDriver
{
public:
  ArmmsDriver();
  void rosControlLoop();

private:
  boost::shared_ptr<armms::ArmmsAPI> comm;
  boost::shared_ptr<ArmmsHardwareInterface> robot;
  boost::shared_ptr<ArmmsBagRecorder> recorder;
  boost::shared_ptr<controller_manager::ControllerManager> cm;
  // boost::shared_ptr<RosInterface> ros_interface;
  // boost::shared_ptr<RpiDiagnostics> rpi_diagnostics;
  boost::shared_ptr<ros::Rate> ros_control_loop_rate;
  boost::shared_ptr<std::thread> ros_control_thread;

  ros::NodeHandle nh_;

  bool flag_reset_controllers;
};

}  // namespace armms_driver