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
#include <boost/thread/mutex.hpp>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Empty.h>

#include "armms_driver/comm/armms_base_comm.h"
#include "armms_driver/comm/armms_fake_comm.h"
#include "armms_driver/comm/armms_kinova_comm.h"
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
  boost::shared_ptr<ArmmsBaseComm> comm;
  boost::shared_ptr<ArmmsHardwareInterface> robot;
  boost::shared_ptr<ArmmsBagRecorder> recorder;
  boost::shared_ptr<controller_manager::ControllerManager> cm;
  // boost::shared_ptr<RosInterface> ros_interface;
  // boost::shared_ptr<RpiDiagnostics> rpi_diagnostics;
  boost::shared_ptr<ros::Rate> ros_control_loop_rate;
  boost::shared_ptr<std::thread> ros_control_thread;

  boost::mutex flag_reset_request_mtx_;
  bool flag_request_ctrl_;

  // boost::mut
  ros::NodeHandle nh_;

  ros::ServiceClient shutdown_srv_;
  ros::ServiceServer reset_controller_service_;
  bool flag_reset_controllers_;
  double ros_control_frequency_;
  bool fake_communication_;
  bool api_logging_;
  std::string device_;

  void initializeServices_();
  void retrieveParameters_();
  bool callbackResetController_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool getResetRequest_();
};

}  // namespace armms_driver