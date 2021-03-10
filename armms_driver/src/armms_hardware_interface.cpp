
#include "armms_driver/armms_hardware_interface.h"

using namespace hardware_interface;
using namespace joint_limits_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace armms_driver
{
ArmmsHardwareInterface::ArmmsHardwareInterface(armms::ArmmsAPI* comm) : comm_(comm)
{
  ROS_INFO("Starting ARMMS Hardware Interface...");
  status_ = OK;
  // Resize vectors
  int num_joints_ = 1;
  joint_names_.resize(num_joints_);
  joint_names_[0] = "joint1";
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // connect and register joint state interface
  JointStateHandle jointStateHandle("joint1", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
  joint_state_interface_.registerHandle(jointStateHandle);

  registerInterface(&joint_state_interface_);

  // connect and register joint position interface
  // Create position joint interface
  JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[0]);
  JointLimits limits;
  SoftJointLimits softLimits;

  getJointLimits("joint1", nh_, limits);

  ros::param::get("/joint_limits/joint1/soft_min_position", softLimits.min_position);
  ros::param::get("/joint_limits/joint1/soft_max_position", softLimits.max_position);
  ros::param::get("/joint_limits/joint1/k_position", softLimits.k_position);
  ros::param::get("/joint_limits/joint1/k_velocity", softLimits.k_velocity);

  PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
  positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
  position_joint_interface_.registerHandle(jointPositionHandle);

  registerInterface(&position_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);

  ROS_INFO_NAMED("ArmmsHardwareInterface", "Interfaces registered.");
}

ArmmsHardwareInterface::~ArmmsHardwareInterface()
{
}

void ArmmsHardwareInterface::setCommandToCurrentPosition()
{
  position_joint_interface_.getHandle("joint1").setCommand(joint_position_[0]);
}

void ArmmsHardwareInterface::read()
{
  float pos = 0;

  if (comm_->initializeActuator(pos) == 0)
  {
    ROS_DEBUG("Read actuator position: %f", pos);
    joint_position_[0] = pos;
    read_error_count_ = 0;
  }
  else
  {
    ROS_WARN("Cannot read actuator position !");
    if (read_error_count_ < 10)
    {
      read_error_count_++;
    }

    return;
  }
}

void ArmmsHardwareInterface::enforceLimit(ros::Duration elapsed_time)
{
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
  status_ = OK;
}

void ArmmsHardwareInterface::resetLimit()
{
  positionJointSoftLimitsInterface.reset();
}

void ArmmsHardwareInterface::write()
{
  float pos = 0, torque = 0;

  if (comm_->setPositionCommandExt(joint_position_command_[0], pos, torque) == 0)
  {
    ROS_DEBUG("Write actuator position command: %f", joint_position_command_[0]);
    joint_position_[0] = pos;
    joint_effort_[0] = torque;
  }
  else
  {
    ROS_WARN("Cannot write actuator position command %f !", joint_position_command_[0]);
    status_ = WRITE_ERROR;
    return;
  }
  status_ = OK;
}

ArmmsHardwareInterface::status_t ArmmsHardwareInterface::getStatus()
{
  if (read_error_count_ >= 10)
  {
    return READ_ERROR;
  }
  return OK;
}

}  // namespace armms_driver