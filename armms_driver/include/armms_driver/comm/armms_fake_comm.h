//============================================================================
// Name        : armms_fake_comm.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_DRIVER_ARMMS_FAKE_COMM_H
#define ARMMS_DRIVER_ARMMS_FAKE_COMM_H

#include <ros/ros.h>
#include <armms_driver/comm/armms_base_comm.h>

namespace armms_driver
{
class ArmmsFakeComm : public ArmmsBaseComm
{
public:
  ArmmsFakeComm();

  int init(const std::string device, const bool& debug_log = false);
  int initializeActuator(float& jointPositionOptical);
  // TODO could be remove
  int clearError();
  int startMotorControl();
  int stopMotorControl();

  int setPositionCommandExt(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                    float& jointSpeed, float& jointTorque, float& jointPMW, float& jointPositionOptical,
                                    short& jointAccelX, short& jointAccelY, short& jointAccelZ, short& jointTemp);
  int setPositionCommand(const float& jointCommand, float& jointCurrent, float& jointPositionHall,
                                 float& jointSpeed, float& jointTorque);
  int getActualPosition(float& jointCurrent, float& jointPositionHall, float& jointSpeed,
                                float& jointTorque);
private:
  double torque_;
  double position_;
  double omega_t_;
};

}  // namespace armms_driver
#endif  // ARMMS_DRIVER_ARMMS_FAKE_COMM_H
