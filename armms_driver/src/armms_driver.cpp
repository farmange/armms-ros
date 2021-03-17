//============================================================================
// Name        : armms_driver.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_driver/armms_driver.h"
#include "armms_msgs/SetInt.h"

namespace armms_driver
{
ArmmsDriver::ArmmsDriver()
{
  retrieveParameters_();
  initializeServices_();

  if (ros_control_frequency_ < 0)
  {
    ROS_ERROR("Sampling frequency could not be lower or equal to zero : %f", ros_control_frequency_);
    ros::shutdown();
    return;
  }

  ros::Duration max_waiting_time = ros::Duration(4.0);
  ros::Duration elapsed_time = ros::Duration(0.0);
  ros::Time init_time = ros::Time::now();
  while (!shutdown_srv_.exists() && elapsed_time < max_waiting_time)
  {
    elapsed_time = (ros::Time::now() - init_time);
  }
  if (!shutdown_srv_.exists())
  {
    ROS_ERROR("Cannot get raspberry pi handle to shutdown properly");
    ros::shutdown();
    return;
  }

  ROS_INFO("Starting ARMMS driver thread (frequency : %fHz)", ros_control_frequency_);

  comm.reset(new armms::ArmmsAPI());

  int init_result = comm->init(device_, api_logging_);

  if (init_result != 0)
  {
    ROS_ERROR("Cannot initialize communication instance...");
    armms_msgs::SetInt msgShutdown;
    msgShutdown.request.value = 1;  // shutdown
    shutdown_srv_.call(msgShutdown);
    ros::shutdown();
    return;
  }
  ROS_INFO("ARMMS communication has been successfully started");

  ros::Duration(0.1).sleep();
  flag_reset_controllers_ = true;

  ROS_INFO("Start hardware control loop");
  ros::Duration(0.5).sleep();

  ROS_INFO("Start hardware interface");
  robot.reset(new ArmmsHardwareInterface(comm.get()));

  ROS_INFO("Start rosbag recorder");
  recorder.reset(new ArmmsBagRecorder(nh_));
  recorder->start();

  ROS_INFO("Create controller manager");
  cm.reset(new controller_manager::ControllerManager(robot.get(), nh_));
  ros::Duration(0.1).sleep();

  ROS_INFO("Starting ros control thread...");
  ros_control_loop_rate.reset(new ros::Rate(ros_control_frequency_));
  ros_control_thread.reset(new std::thread(boost::bind(&ArmmsDriver::rosControlLoop, this)));
}

void ArmmsDriver::rosControlLoop()
{
  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time;

  while (ros::ok())
  {
    if (robot->getStatus() != ArmmsHardwareInterface::OK)
    {
      flag_reset_controllers_ = true;
    }

    if (getResetRequest_())
    {
      flag_reset_controllers_ = true;
    }

    robot->read();

    current_time = ros::Time::now();
    elapsed_time = ros::Duration(current_time - last_time);
    last_time = current_time;

    if (flag_reset_controllers_)
    {
      ROS_DEBUG("Reset controller...");
      robot->setCommandToCurrentPosition();
      robot->resetLimit();
      cm->update(ros::Time::now(), elapsed_time, true);
      flag_reset_controllers_ = false;
    }
    else
    {
      cm->update(ros::Time::now(), elapsed_time, false);
      robot->enforceLimit(elapsed_time);
    }

    robot->write();

    ros_control_loop_rate->sleep();
  }
}

void ArmmsDriver::initializeServices_()
{
  shutdown_srv_ = nh_.serviceClient<armms_msgs::SetInt>("/armms_rpi/shutdown_rpi");
  reset_controller_service_ =
      nh_.advertiseService("/armms_driver/reset_controller", &ArmmsDriver::callbackResetController_, this);
}

void ArmmsDriver::retrieveParameters_()
{
  ros_control_frequency_ = 0;
  api_logging_ = false;
  device_ = "";
  ros::param::get("~ros_control_loop_frequency", ros_control_frequency_);
  ros::param::get("~api_logging", api_logging_);
  ros::param::get("~device", device_);
}

bool ArmmsDriver::callbackResetController_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  flag_reset_request_mtx_.lock();
  flag_request_ctrl_ = true;
  flag_reset_request_mtx_.unlock();
  return true;
}

bool ArmmsDriver::getResetRequest_()
{
  flag_reset_request_mtx_.lock();
  bool flag_copy = flag_request_ctrl_;
  flag_request_ctrl_ = false;
  flag_reset_request_mtx_.unlock();
  return flag_copy;
}

}  // namespace armms_driver