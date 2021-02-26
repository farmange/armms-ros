#ifndef ROS_CONTROL__ARMMS_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ARMMS_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <mutex>  // std::mutex

#include "std_srvs/Empty.h"

// #include <ROBOTcpp/ROBOT.h>
#include <armms_hardware_interface/armms_hardware.h>
#include "armms_hardware_interface/armms_api.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace armms_hardware_interface
{
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class ArmmsHardwareInterface : public armms_hardware_interface::ArmmsHardware
{
public:
  ArmmsHardwareInterface(armms::ArmmsAPI* comm);
  ~ArmmsHardwareInterface();
  // void init();
  // void initPosition();
  // void update(const ros::TimerEvent& e);
  void read();
  void enforceLimit(ros::Duration elapsed_time);
  void write();

private:
  ros::NodeHandle nh_;
  armms::ArmmsAPI* comm_;
  ros::ServiceServer start_control_service_;
  ros::ServiceServer stop_control_service_;

  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
};

}  // namespace armms_hardware_interface

#endif