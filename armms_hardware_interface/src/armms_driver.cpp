//============================================================================
// Name        : armms_driver.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
/*
    niryo_one_driver_node.cpp
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/shared_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>

// #include "niryo_one_driver/niryo_one_hardware_interface.h"
// #include "niryo_one_driver/communication_base.h"
// #include "niryo_one_driver/niryo_one_communication.h"
// #include "niryo_one_driver/fake_communication.h"
// #include "niryo_one_driver/ros_interface.h"
#include "armms_hardware_interface/armms_api.h"
#include "armms_hardware_interface/armms_hardware_interface.h"

#include "std_msgs/Empty.h"

using namespace armms_hardware_interface;

class ArmmsDriver
{
private:
  boost::shared_ptr<armms::ArmmsAPI> comm;
  boost::shared_ptr<ArmmsHardwareInterface> robot;
  boost::shared_ptr<controller_manager::ControllerManager> cm;
  // boost::shared_ptr<RosInterface> ros_interface;
  // boost::shared_ptr<RpiDiagnostics> rpi_diagnostics;
  boost::shared_ptr<ros::Rate> ros_control_loop_rate;
  boost::shared_ptr<std::thread> ros_control_thread;

  ros::NodeHandle nh_;

  bool flag_reset_controllers;

  // ros::Subscriber reset_controller_subscriber; // workaround to compensate missed steps
  // ros::Subscriber trajectory_result_subscriber;

public:
  void rosControlLoop()
  {
    ROS_INFO_NAMED("ArmmsDriver", "rosControlLoop");

    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time;
    ROS_INFO_NAMED("ArmmsDriver", "while");

    while (ros::ok())
    {
      ROS_DEBUG_NAMED("ArmmsDriver", "Read...");

      robot->read();
      if (robot->getStatus() == ArmmsHardwareInterface::OK)
      {
        ROS_DEBUG_NAMED("ArmmsDriver", "Read OK");

        current_time = ros::Time::now();
        elapsed_time = ros::Duration(current_time - last_time);
        last_time = current_time;

        if (flag_reset_controllers)
        {
          // robot->setCommandToCurrentPosition();
          cm->update(ros::Time::now(), elapsed_time, true);
          flag_reset_controllers = false;
        }
        else
        {
          cm->update(ros::Time::now(), elapsed_time, false);
        }
        robot->enforceLimit(elapsed_time);

        robot->write();
      }
      ros_control_loop_rate->sleep();
    }
  }

  ArmmsDriver()
  {
    double ros_control_frequency;
    bool api_logging = false;
    std::string device = "";
    ros::param::get("~ros_control_loop_frequency", ros_control_frequency);
    ros::param::get("~api_logging", api_logging);
    ros::param::get("~device", device);

    // nh_.getParam("~ros_control_loop_frequency", ros_control_frequency);
    // nh_.getParam("api_logging", api_logging);
    // nh_.getParam("device", device);

    ROS_INFO("Starting ARMMS driver thread (frequency : %lf)", ros_control_frequency);

    comm.reset(new armms::ArmmsAPI());

    int init_result = comm->init(device, api_logging);
    if (init_result != 0)
    {
      return;  // need to check last ROS_ERROR to get more info
    }

    ROS_INFO("ARMMS communication has been successfully started");

    ros::Duration(0.1).sleep();
    flag_reset_controllers = true;

    ROS_INFO("Start hardware control loop");
    // comm->manageHardwareConnection();
    ros::Duration(0.5).sleep();

    ROS_INFO("Start hardware interface");
    robot.reset(new ArmmsHardwareInterface(comm.get()));

    ROS_INFO("Create controller manager");
    cm.reset(new controller_manager::ControllerManager(robot.get(), nh_));
    ros::Duration(0.1).sleep();

    ROS_INFO("Starting ros control thread...");
    ros_control_loop_rate.reset(new ros::Rate(ros_control_frequency));
    ros_control_thread.reset(new std::thread(boost::bind(&ArmmsDriver::rosControlLoop, this)));

    // ROS_INFO("Start Rpi Diagnostics...");
    // rpi_diagnostics.reset(new RpiDiagnostics());

    // ROS_INFO("Starting ROS interface...");
    // bool learning_mode_activated_on_startup = true;
    // ros_interface.reset(new RosInterface(comm.get(), rpi_diagnostics.get(), &flag_reset_controllers,
    //                                      learning_mode_activated_on_startup, hardware_version));

    // // activate learning mode
    // comm->activateLearningMode(learning_mode_activated_on_startup);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_driver");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ArmmsDriver nd;

  ros::waitForShutdown();

  ROS_INFO("shutdown node");
}
