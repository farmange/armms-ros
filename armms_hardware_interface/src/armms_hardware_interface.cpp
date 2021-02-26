#include <sstream>
#include <armms_hardware_interface/armms_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
// #include <ROBOTcpp/ROBOT.h>

using namespace hardware_interface;
using namespace joint_limits_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace armms_hardware_interface
{
ArmmsHardwareInterface::ArmmsHardwareInterface(armms::ArmmsAPI* comm) : comm_(comm)
{
  ROS_INFO("Starting ARMMS Hardware Interface...");

  // Resize vectors
  int num_joints_ = 1;
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
  PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
  positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
  position_joint_interface_.registerHandle(jointPositionHandle);

  registerInterface(&position_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);

  ROS_INFO("Interfaces registered.");
}

ArmmsHardwareInterface::~ArmmsHardwareInterface()
{
}

void ArmmsHardwareInterface::read()
{
  ROS_DEBUG_STREAM_NAMED("ArmmsHardwareInterface", "read");

  float pos = 0;
  if (comm_->initializeActuator(pos) == 0)
  {
    ROS_DEBUG_STREAM_NAMED("ArmmsHardwareInterface", "Read position: " << pos);
    joint_position_[0] = pos;
  }
  else
  {
    ROS_WARN_STREAM_NAMED("ArmmsHardwareInterface", "Problem reading the position of " << joint_names_[0] << " !");
  }
}

void ArmmsHardwareInterface::enforceLimit(ros::Duration elapsed_time)
{
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
}

void ArmmsHardwareInterface::write()
{
  ROS_DEBUG_STREAM_NAMED("ArmmsHardwareInterface", "write");
  float pos = 0, torque = 0;

  if (comm_->setPositionCommandExt(joint_position_command_[0], pos, torque) == 0)
  {
    ROS_DEBUG_STREAM_NAMED("ArmmsHardwareInterface", "Write position command: " << joint_position_command_[0]);
    ROS_DEBUG_NAMED("ArmmsHardwareInterface", "Read position : %f | torque : %f", pos, torque);
    joint_position_[0] = pos;
    joint_effort_[0] = torque;
  }
  else
  {
    ROS_WARN_STREAM_NAMED("ArmmsHardwareInterface", "Problem writing the position command of "
                                                        << joint_names_[0] << " : " << joint_position_command_[0]
                                                        << "!");
  }
}
}  // namespace armms_hardware_interface