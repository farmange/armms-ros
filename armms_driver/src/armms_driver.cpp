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
  double ros_control_frequency;
  bool api_logging = false;
  std::string device = "";
  ros::param::get("~ros_control_loop_frequency", ros_control_frequency);
  ros::param::get("~api_logging", api_logging);
  ros::param::get("~device", device);

  ros::ServiceClient shutdown_srv_ = nh_.serviceClient<armms_msgs::SetInt>("/armms_rpi/shutdown_rpi");
  ros::Duration max_waiting_time = ros::Duration(4.0);
  ros::Duration elapsed_time = ros::Duration(0.0);
  ros::Time init_time = ros::Time::now();
  while (!shutdown_srv_.exists() && elapsed_time < max_waiting_time)
  {
    elapsed_time = (ros::Time::now() - init_time);
  }
  if (!shutdown_srv_.exists())
  {
    ROS_INFO("Cannot get raspberry pi handle to shutdown properly");
    ros::shutdown();
    return;
  }

  ROS_INFO("Starting ARMMS driver thread (frequency : %fHz)", ros_control_frequency);

  comm.reset(new armms::ArmmsAPI());

  int init_result = comm->init(device, api_logging);

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
  flag_reset_controllers = true;

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
  ros_control_loop_rate.reset(new ros::Rate(ros_control_frequency));
  ros_control_thread.reset(new std::thread(boost::bind(&ArmmsDriver::rosControlLoop, this)));
}

void ArmmsDriver::rosControlLoop()
{
  ROS_INFO_NAMED("ArmmsDriver", "rosControlLoop");

  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time;

  while (ros::ok())
  {
    if (robot->getStatus() != ArmmsHardwareInterface::OK)
    {
      flag_reset_controllers = true;
    }

    robot->read();

    current_time = ros::Time::now();
    elapsed_time = ros::Duration(current_time - last_time);
    last_time = current_time;

    if (flag_reset_controllers)
    {
      ROS_DEBUG_NAMED("ArmmsDriver", "Reset controller...");
      robot->setCommandToCurrentPosition();
      robot->resetLimit();
      cm->update(ros::Time::now(), elapsed_time, true);
      flag_reset_controllers = false;
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

}  // namespace armms_driver