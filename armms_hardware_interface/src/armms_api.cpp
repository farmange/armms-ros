//============================================================================
// Name        : armms_api.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_hardware_interface/armms_api.h"
#include "kinovadrv/kinovadrv.h"

using namespace KinovaApi;

namespace armms
{
ArmmsAPI::ArmmsAPI(ros::NodeHandle nh) : nh_(nh)
{
}

int ArmmsAPI::init(const std::string device, const bool& debug_log)
{
  ROS_INFO("ArmmsAPI::init");
  if (loadLibrary("libkinovadrv.so") != 0)
  {
    ROS_FATAL("Failed to load library libkinovadrv.so !");
    return 1;
  }

  if (driver_->init(device, debug_log) != APILayer::API_OK)
  {
    ROS_FATAL("Failed to initialize driver !");
    return 1;
  }
  ROS_INFO("Driver initialisation done.");

  int jointAddressParam = 0;
  ros::param::get("/armms_hardware_interface/actuator_address", jointAddressParam);
  jointAddress_ = static_cast<uint16_t>(jointAddressParam);

  return 0;
}

// returns 0 if robot connection sucessful
int ArmmsAPI::loadLibrary(const char* comm_lib)
{
  void* driver_command_lib = dlopen(comm_lib, RTLD_NOW);
  if (driver_command_lib == NULL)
  {
    ROS_FATAL("Failed to load library with dlopen (code: %s) !", dlerror());
    return 1;
  }
  // driver_command_lib_ APILayer api;
  void* create_api = dlsym(driver_command_lib, "create");
  // create an instance of the class
  ApiFactory* factory = (ApiFactory*)dlsym(driver_command_lib, "ApiFactory");
  driver_ = factory->makedyn();
  if (!driver_)
  {
    ROS_FATAL("Failed to load symbol with dlsym (code: %s) !", dlerror());
    return 1;
  }
  ROS_INFO("Library %s correctly loaded.", comm_lib);

  return 0;
}

// TODO documentation
int ArmmsAPI::setPositionCommandExt(const float& jointCommand, float& jointPositionOptical, float& jointTorque)
{
  float jointCurrent = 0.0;
  float jointPositionHall = 0.0;
  float jointSpeed = 0.0;
  // float jointTorque = 0.0;
  float jointPMW = 0.0;
  // float jointPositionOptical = 0.0;
  short jointAccelX = 0;
  short jointAccelY = 0;
  short jointAccelZ = 0;
  short jointTemp = 0;
  APILayer::ApiStatus_t status =
      driver_->setCommandAllValue(jointAddress_, jointCommand, jointCurrent, jointPositionHall, jointSpeed, jointTorque,
                                  jointPMW, jointPositionOptical, jointAccelX, jointAccelY, jointAccelZ, jointTemp);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

// TODO documentation
int ArmmsAPI::setPositionCommand(const float& jointCommand, float& jointPositionOptical, float& jointTorque)
{
  float jointCurrent = 0.0;
  float jointPositionHall = 0.0;
  float jointSpeed = 0.0;
  APILayer::ApiStatus_t status = driver_->setPositionCommand(jointAddress_, jointCommand, jointCurrent,
                                                             jointPositionHall, jointSpeed, jointTorque);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsAPI::initializeActuator(float& jointPositionOptical)
{
  APILayer::ApiStatus_t status;
  status = driver_->deviceInitialisation(jointAddress_, jointPositionOptical);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsAPI::clearError()
{
  APILayer::ApiStatus_t status;
  status = driver_->clearError(jointAddress_);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsAPI::startMotorControl()
{
  APILayer::ApiStatus_t status;
  status = driver_->startMotorControl(jointAddress_);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsAPI::stopMotorControl()
{
  APILayer::ApiStatus_t status;
  status = driver_->stopMotorControl(jointAddress_);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

// TODO documentation
int ArmmsAPI::getJointStatesSetCommand(const float& jointCommand, float& jointPositionOptical, float& jointSpeed,
                                       float& jointTorque)
{
  // float jointCommand = 0.0;
  float jointCurrent = 0.0;
  float jointPositionHall = 0.0;
  // float jointSpeed = 0.0;
  // float jointTorque = 0.0;
  float jointPMW = 0.0;
  // float jointPositionOptical = 0.0;
  short jointAccelX = 0;
  short jointAccelY = 0;
  short jointAccelZ = 0;
  short jointTemp = 0;

  APILayer::ApiStatus_t status =
      driver_->setCommandAllValue(jointAddress_, jointCommand, jointCurrent, jointPositionHall, jointSpeed, jointTorque,
                                  jointPMW, jointPositionOptical, jointAccelX, jointAccelY, jointAccelZ, jointTemp);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}
}  // namespace armms
