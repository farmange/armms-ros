#ifndef ARMMS_DRIVER_ARMMS_HARDWARE_INTERFACE_H
#define ARMMS_DRIVER_ARMMS_HARDWARE_INTERFACE_H

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
#include <mutex>
#include "std_srvs/Empty.h"

#include "armms_driver/armms_hardware.h"
#include "armms_driver/comm/armms_base_comm.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace armms_driver
{
class ArmmsHardwareInterface : public armms_driver::ArmmsHardware
{
public:
  typedef enum status_e
  {
    OK = 0,
    READ_ERROR,
    WRITE_ERROR
  } status_t;
  ArmmsHardwareInterface(ArmmsBaseComm* comm);
  ~ArmmsHardwareInterface();
  // void init();
  // void initPosition();
  // void update(const ros::TimerEvent& e);
  void setCommandToCurrentPosition();
  void read();
  void enforceLimit(ros::Duration elapsed_time);
  void resetLimit();
  void write();
  status_t getStatus();

private:
  ros::NodeHandle nh_;
  ArmmsBaseComm* comm_;
  ros::ServiceServer start_control_service_;
  ros::ServiceServer stop_control_service_;
  status_t status_;
  int read_error_count_;
  int write_error_count_;
  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
};

}  // namespace armms_driver

#endif