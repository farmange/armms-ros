//============================================================================
// Name        : armms_driver_node.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : A ROS driver for controlling the ARMMS robotic components
//============================================================================

// #include "kinovadrv/kinovadrv.h"
#include "armms_driver/armms_api.h"

// #include "kinova_driver/kinova_tool_pose_action.h"
// #include "kinova_driver/kinova_joint_angles_action.h"
// #include "kinova_driver/kinova_fingers_action.h"
// #include "kinova_driver/kinova_joint_trajectory_controller.h"

#include <ros/ros.h>
#include <boost/thread/recursive_mutex.hpp>

// #include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_driver_node");
  ros::NodeHandle nh("~");
  boost::recursive_mutex api_mutex;

  bool is_first_init = true;
  std::string armms_robotType = "";
  std::string armms_robotName = "";

  // Retrieve the (non-option) argument:
  if ((argc <= 1) || (argv[argc - 1] == NULL))  // there is NO input...
  {
    std::cerr << "No armms_robotType provided in the argument!" << std::endl;
    return -1;
  }
  else  // there is an input...
  {
    armms_robotType = argv[argc - 1];
    ROS_INFO("armms_robotType is %s.", armms_robotType.c_str());
    if (!nh.getParam("robot_name", armms_robotName))
    {
      armms_robotName = armms_robotType;
    }
    ROS_INFO("armms_robotName is %s.", armms_robotName.c_str());
  }

  while (ros::ok())
  {
    // try
    // {
    //   kinova::KinovaComm comm(nh, api_mutex, is_first_init, kinova_robotType);
    //   kinova::KinovaArm kinova_arm(comm, nh, kinova_robotType, kinova_robotName);
    //   kinova::KinovaPoseActionServer pose_server(comm, nh, kinova_robotType, kinova_robotName);
    //   kinova::KinovaAnglesActionServer angles_server(comm, nh);
    //   kinova::KinovaFingersActionServer fingers_server(comm, nh);
    //   kinova::JointTrajectoryController joint_trajectory_controller(comm, nh);
    //   ros::spin();
    // }
    // catch (const std::exception& e)
    // {
    //   ROS_ERROR_STREAM(e.what());
    //   kinova::KinovaAPI api;
    //   boost::recursive_mutex::scoped_lock lock(api_mutex);
    //   api.closeAPI();
    //   ros::Duration(1.0).sleep();
    // }

    // armms::ArmmsAPI api;
    // api.init();
    // ros::spin();

    is_first_init = false;
  }
  return 0;
}
