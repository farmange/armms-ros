//============================================================================
// Name        : armms_kinova_comm.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "armms_driver/comm/armms_kinova_comm.h"
#include "kinovadrv/kinovadrv.h"

using namespace KinovaApi;

namespace armms_driver
{
ArmmsKinovaComm::ArmmsKinovaComm()
{
}

int ArmmsKinovaComm::init(const std::string device, const bool& debug_log)
{
  if (loadLibrary_("libkinovadrv.so") != 0)
  {
    ROS_FATAL("Failed to load library libkinovadrv.so !");
    return 1;
  }

  if (driver_->init(device, debug_log) != APILayer::API_OK)
  {
    ROS_FATAL("Failed to initialize driver (device: %s)!", device.c_str());
    return 1;
  }
  ROS_INFO("Driver initialisation done.");

  int jointAddressParam = 0;
  // ros::param::get("/armms_hardware_interface/actuator_address", jointAddressParam);
  ros::param::get("~actuator_address", jointAddressParam);

  jointAddress_ = static_cast<uint16_t>(jointAddressParam);

  return 0;
}

// returns 0 if robot connection sucessful
int ArmmsKinovaComm::loadLibrary_(const char* comm_lib)
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
int ArmmsKinovaComm::setPositionCommandExt(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                           float& jointSpeed, float& jointTorque, float& jointPMW,
                                           float& jointPositionOptical, short& jointAccelX, short& jointAccelY,
                                           short& jointAccelZ, short& jointTemp)
{
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
int ArmmsKinovaComm::setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                        float& jointSpeed, float& jointTorque)
{
  APILayer::ApiStatus_t status = driver_->setPositionCommand(jointAddress_, jointCommand, jointCurrent,
                                                             jointPositionHall, jointSpeed, jointTorque);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsKinovaComm::initializeActuator(float& jointPositionOptical)
{
  APILayer::ApiStatus_t status;
  status = driver_->deviceInitialisation(jointAddress_, jointPositionOptical);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsKinovaComm::clearError()
{
  APILayer::ApiStatus_t status;
  status = driver_->clearError(jointAddress_);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsKinovaComm::startMotorControl()
{
  APILayer::ApiStatus_t status;
  status = driver_->startMotorControl(jointAddress_);

  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

int ArmmsKinovaComm::stopMotorControl()
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
int ArmmsKinovaComm::getActualPosition(float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                                       float& jointTorque)
{
  APILayer::ApiStatus_t status;
  status = driver_->getActualPosition(jointAddress_, jointCurrent, jointPositionHall, jointSpeed, jointTorque);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}
}  // namespace armms_driver
