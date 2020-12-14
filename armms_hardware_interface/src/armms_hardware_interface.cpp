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
ArmmsHardwareInterface::ArmmsHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
{
  init();
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  nh_.param("/at1x/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &ArmmsHardwareInterface::update, this);
}

ArmmsHardwareInterface::~ArmmsHardwareInterface()
{
}

void ArmmsHardwareInterface::init()
{
  // Get joint names
  nh_.getParam("/at1x/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();
  ROS_DEBUG_STREAM("Initialize ArmmsHardwareInterface with " << num_joints_ << " joints");

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize Controller
  for (int i = 0; i < num_joints_; ++i)
  {
    // ROBOTcpp::Joint joint = ROBOT.getJoint(joint_names_[i]);
    std::string jointname = joint_names_[i];
    ROS_DEBUG_STREAM("Initialize joint : " << jointname);

    // Create joint state interface
    JointStateHandle jointStateHandle(jointname, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
    JointLimits limits;
    SoftJointLimits softLimits;
    getJointLimits(jointname, nh_, limits);
    PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
    positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
    position_joint_interface_.registerHandle(jointPositionHandle);

    // Create effort joint interface
    JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
    effort_joint_interface_.registerHandle(jointEffortHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);

  ROS_DEBUG_STREAM("Initialize ArmmsAPI...");
  api_.init();
  read();
  for (int i = 0; i < num_joints_; i++)
  {
    joint_position_command_[i] = joint_position_[i];
  }
}

void ArmmsHardwareInterface::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void ArmmsHardwareInterface::read()
{
  for (int i = 0; i < num_joints_; i++)
  {
    // joint_position_[i] = joint_position_[i] + 1;
    float pos = 0;
    if (api_.setPositionCommand(joint_position_command_[i], pos) == 0)
    {
      ROS_DEBUG_STREAM("Read position: " << pos);
      joint_position_[i] = pos;
    }
    else
    {
      ROS_WARN_STREAM("Problem reading the position of " << joint_names_[i] << " !");
    }
  }
}

void ArmmsHardwareInterface::write(ros::Duration elapsed_time)
{
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
  for (int i = 0; i < num_joints_; i++)
  {
    // ROBOT.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
    float pos = 0;

    if (api_.setPositionCommand(joint_position_command_[i], pos) == 0)
    {
      ROS_DEBUG_STREAM("Write position command: " << joint_position_command_[i]);
    }
    else
    {
      ROS_WARN_STREAM("Problem writing the position command of " << joint_names_[i] << " : "
                                                                 << joint_position_command_[i] << "!");
    }
  }
}
}  // namespace armms_hardware_interface