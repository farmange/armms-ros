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
ArmmsAPI::ArmmsAPI()
{
  // init();
}

int ArmmsAPI::init()
{
  if (loadLibrary("libkinovadrv.so") != 0)
  {
    ros::shutdown();
    exit(1);
  }

  Api_->init();
  return 0;
}

// returns 0 if robot connection sucessful
int ArmmsAPI::loadLibrary(const char* comm_lib)
{
  void* API_command_lib = dlopen(comm_lib, RTLD_NOW);
  if (API_command_lib == NULL)
  {
    ROS_FATAL("Failed to load library with dlopen (code: %s) !", dlerror());
    return 1;
  }
  // API_command_lib_ APILayer api;
  void* create_api = dlsym(API_command_lib, "create");
  // create an instance of the class
  ApiFactory* factory = (ApiFactory*)dlsym(API_command_lib, "ApiFactory");
  Api_ = factory->makedyn();
  if (!Api_)
  {
    ROS_FATAL("Failed to load symbol with dlsym (code: %s) !", dlerror());
    return 1;
  }
  return 0;
}

// TODO documentation
int ArmmsAPI::setPositionCommand(const float& jointCommand, float& jointPositionOptical)
{
  const uint16_t jointAddress = 0x11;

  float jointCurrent = 0.0;
  float jointPositionHall = 0.0;
  float jointSpeed = 0.0;
  float jointTorque = 0.0;
  float jointPMW = 0.0;
  // float jointPositionOptical = 0.0;
  short jointAccelX = 0;
  short jointAccelY = 0;
  short jointAccelZ = 0;
  short jointTemp = 0;
  APILayer::ApiStatus_t status =
      Api_->setCommandAllValue(jointAddress, jointCommand, jointCurrent, jointPositionHall, jointSpeed, jointTorque,
                               jointPMW, jointPositionOptical, jointAccelX, jointAccelY, jointAccelZ, jointTemp);
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
  const uint16_t jointAddress = 0x11;

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
      Api_->setCommandAllValue(jointAddress, jointCommand, jointCurrent, jointPositionHall, jointSpeed, jointTorque,
                               jointPMW, jointPositionOptical, jointAccelX, jointAccelY, jointAccelZ, jointTemp);
  if (status == APILayer::API_OK)
  {
    return 0;
  }
  return 1;
}

// void* ArmmsAPI::initCommandLayerFunction(const char* name)
// {
//   char functionName[100];
//   strcpy(functionName, name);
//   if (API_type_ == ETHERNET)
//   {
//     strcpy(functionName, "Ethernet_");
//     strcat(functionName, name);
//   }
//   void* function_pointer = dlsym(API_command_lib_, functionName);
//   assert(function_pointer != NULL);
//   return function_pointer;
// }

// void* KinovaAPI::initCommLayerFunction(const char* name)
// {
//   char functionName[100];
//   strcpy(functionName, name);
//   if (API_type_ == ETHERNET)
//   {
//     strcpy(functionName, "Ethernet_Communication_");
//     strcat(functionName, name);
//   }
//   void* function_pointer = dlsym(kinova_comm_lib_, name);
//   assert(function_pointer != NULL);
//   return function_pointer;
// }

// int KinovaAPI::initializeKinovaAPIFunctions(KinovaAPIType connection_type)
// {
//   // try USB connection
//   API_type_ = connection_type;

//   if (API_type_ == USB)
//   {
//     loadLibraries(KINOVA_USB_LIBRARY, KINOVA_COMM_USB_LIBRARY);
//   }
//   else
//   {
//     loadLibraries(KINOVA_ETH_LIBRARY, KINOVA_COMM_ETH_LIBRARY);
//   }

//   // %Tag(general function)%

//   initAPI = (int (*)())initCommandLayerFunction("InitAPI");

//   if (API_type_ != USB)
//   {
//     initEthernetAPI = (int (*)(EthernetCommConfig&))initCommandLayerFunction("InitEthernetAPI");
//   }

//   closeAPI = (int (*)())initCommandLayerFunction("CloseAPI");

//   startControlAPI = (int (*)())initCommandLayerFunction("StartControlAPI");

//   stopControlAPI = (int (*)())initCommandLayerFunction("StopControlAPI");

//   startForceControl = (int (*)())initCommandLayerFunction("StartForceControl");

//   stopForceControl = (int (*)())initCommandLayerFunction("StopForceControl");

//   restoreFactoryDefault = (int (*)())initCommandLayerFunction("RestoreFactoryDefault");

//   sendJoystickCommand = (int (*)(JoystickCommand))initCommandLayerFunction("SendJoystickCommand");

//   getJoystickValue = (int (*)(JoystickCommand&))initCommandLayerFunction("GetJoystickValue");

//   getCodeVersion = (int (*)(int[CODE_VERSION_COUNT]))initCommandLayerFunction("GetCodeVersion");

//   getAPIVersion = (int (*)(int[API_VERSION_COUNT]))initCommandLayerFunction("GetAPIVersion");

//   // not working with Ethernet API
//   // getDeviceCount = (int (*)(int &))initCommLayerFunction("GetDeviceCount");

//   getDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int&))initCommandLayerFunction("GetDevices");

//   setActiveDevice = (int (*)(KinovaDevice))initCommandLayerFunction("SetActiveDevice");

//   refresDevicesList = (int (*)())initCommandLayerFunction("RefresDevicesList");

//   getControlType = (int (*)(int&))initCommandLayerFunction("GetControlType");

//   getClientConfigurations = (int (*)(ClientConfigurations&))initCommandLayerFunction("GetClientConfigurations");

//   setClientConfigurations = (int (*)(ClientConfigurations))initCommandLayerFunction("SetClientConfigurations");

//   setFrameType = (int (*)(int))initCommandLayerFunction("SetFrameType");

//   startCurrentLimitation = (int (*)())initCommandLayerFunction("StartCurrentLimitation");

//   stopCurrentLimitation = (int (*)())initCommandLayerFunction("StopCurrentLimitation");

//   getGeneralInformations = (int (*)(GeneralInformations&))initCommandLayerFunction("GetGeneralInformations");

//   getQuickStatus = (int (*)(QuickStatus&))initCommandLayerFunction("GetQuickStatus");

//   getForcesInfo = (int (*)(ForcesInfo&))initCommandLayerFunction("GetForcesInfo");

//   getSensorsInfo = (int (*)(SensorsInfo&))initCommandLayerFunction("GetSensorsInfo");

//   getGripperStatus = (int (*)(Gripper&))initCommandLayerFunction("GetGripperStatus");

//   getCommandVelocity = (int (*)(float[CARTESIAN_SIZE],
//   float[MAX_ACTUATORS]))initCommandLayerFunction("GetCommandVeloci"
//                                                                                                       "ty");

//   // %EndTag(general function)%

//   // %Tag(joint angular)%

//   setAngularControl = (int (*)())initCommandLayerFunction("SetAngularControl");

//   getAngularCommand = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularCommand");

//   getAngularPosition = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularPosition");

//   getAngularVelocity = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularVelocity");

//   getAngularAcceleration = (int (*)(AngularAcceleration&))initCommandLayerFunction("GetActuatorAcceleration");

//   getAngularForce = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularForce");

//   getAngularCurrent = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularCurrent");

//   getAngularCurrentMotor = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularCurrentMotor");

//   getPositionCurrentActuators = (int
//   (*)(float[POSITION_CURRENT_COUNT]))initCommandLayerFunction("GetPositionCurrentAct"
//                                                                                                  "uators");

//   setActuatorPID = (int (*)(unsigned int, float, float, float))initCommandLayerFunction("SetActuatorPID");

//   // %EndTag(joint angular)%

//   // %Tag(tool cartesian)%

//   setCartesianControl = (int (*)())initCommandLayerFunction("SetCartesianControl");

//   getCartesianCommand = (int (*)(CartesianPosition&))initCommandLayerFunction("GetCartesianCommand");

//   getCartesianPosition = (int (*)(CartesianPosition&))initCommandLayerFunction("GetCartesianPosition");

//   getCartesianForce = (int (*)(CartesianPosition&))initCommandLayerFunction("GetCartesianForce");

//   setCartesianForceMinMax = (int (*)(CartesianInfo,
//   CartesianInfo))initCommandLayerFunction("SetCartesianForceMinMax");

//   setCartesianInertiaDamping = (int (*)(CartesianInfo,
//   CartesianInfo))initCommandLayerFunction("SetCartesianInertiaDamp"
//                                                                                                "ing");

//   setEndEffectorOffset = (int (*)(unsigned int, float, float,
//   float))initCommandLayerFunction("SetEndEffectorOffset");

//   getEndEffectorOffset = (int (*)(unsigned int&, float&, float&,
//   float&))initCommandLayerFunction("GetEndEffectorOffse"
//                                                                                                   "t");

//   getActualTrajectoryInfo = (int (*)(TrajectoryPoint&))initCommandLayerFunction("GetActualTrajectoryInfo");

//   getGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO&))initCommandLayerFunction("GetGlobalTrajectoryInfo");

//   sendAdvanceTrajectory = (int (*)(TrajectoryPoint))initCommandLayerFunction("SendAdvanceTrajectory");

//   sendBasicTrajectory = (int (*)(TrajectoryPoint))initCommandLayerFunction("SendBasicTrajectory");

//   eraseAllTrajectories = (int (*)())initCommandLayerFunction("EraseAllTrajectories");

//   eraseAllProtectionZones = (int (*)())initCommandLayerFunction("EraseAllProtectionZones");

//   getProtectionZone = (int (*)(ZoneList&))initCommandLayerFunction("GetProtectionZone");

//   setProtectionZone = (int (*)(ZoneList))initCommandLayerFunction("SetProtectionZone");

//   StartRedundantJointNullSpaceMotion = (int (*)())initCommandLayerFunction("StartRedundantJointNullSpaceMotion");

//   StopRedundantJointNullSpaceMotion = (int (*)())initCommandLayerFunction("StopRedundantJointNullSpaceMotion");

//   ActivateCollisionAutomaticAvoidance = (int
//   (*)(int))initCommandLayerFunction("ActivateCollisionAutomaticAvoidance");

//   // ActivateSingularityAutomaticAvoidance = (int
//   // (*)(int))initCommandLayerFunction("ActivateSingularityAutomaticAvoidance");

//   // ActivateAutoNullSpaceMotionCartesian = (int
//   // (*)(int))initCommandLayerFunction("ActivateAutoNullSpaceMotionCartesian");

//   // %EndTag(tool cartesian)%

//   // %Tag(torque control)%

//   switchTrajectoryTorque = (int (*)(GENERALCONTROL_TYPE))initCommandLayerFunction("SwitchTrajectoryTorque");

//   sendAngularTorqueCommand = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SendAngularTorqueCommand");

//   setTorqueZero = (int (*)(int))initCommandLayerFunction("SetTorqueZero");

//   sendCartesianForceCommand = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SendCartesianForceCommand");

//   // Toque parameters
//   setGravityPayload = (int (*)(float[GRAVITY_PAYLOAD_SIZE]))initCommandLayerFunction("SetGravityPayload");

//   setAngularTorqueMinMax = (int (*)(AngularInfo, AngularInfo))initCommandLayerFunction("SetAngularTorqueMinMax");

//   setTorqueSafetyFactor = (int (*)(float))initCommandLayerFunction("SetTorqueSafetyFactor");

//   setGravityManualInputParam = (int (*)(float
//   command[GRAVITY_PARAM_SIZE]))initCommandLayerFunction("SetGravityManualIn"
//                                                                                                     "putParam");

//   setGravityOptimalZParam = (int (*)(float[GRAVITY_PARAM_SIZE]))initCommandLayerFunction("SetGravityOptimalZParam");

//   runGravityZEstimationSequence = (int (*)(ROBOT_TYPE,
//   double[OPTIMAL_Z_PARAM_SIZE]))initCommandLayerFunction("RunGravi"
//                                                                                                               "tyZEstim"
//                                                                                                               "ationSeq"
//                                                                                                               "uence");

//   runGravityZEstimationSequence7DOF =
//       (int (*)(ROBOT_TYPE,
//       float[OPTIMAL_Z_PARAM_SIZE_7DOF]))initCommandLayerFunction("RunGravityZEstimationSequence7DO"
//                                                                                       "F");

//   // %EndTag(torque control)%

//   // %Tag(pre-defined)%

//   moveHome = (int (*)())initCommandLayerFunction("MoveHome");

//   initFingers = (int (*)())initCommandLayerFunction("InitFingers");

//   // %EndTag(pre-defined)%

//   // The following APIs are not wrapped in kinova_comm, users should call kinova_api with extra caution.

//   // %Tag(less interest in ROS)%

//   setControlMapping = (int (*)(ControlMappingCharts))initCommandLayerFunction("SetControlMapping");

//   getControlMapping = (int (*)(ControlMappingCharts&))initCommandLayerFunction("GetControlMapping");

//   // %EndTag(less interest in ROS)%

//   // %Tag(not function, place holder, etc)%

//   getSingularityVector = (int (*)(SingularityVector&))initCommandLayerFunction(
//       "SendCartesianForceCommand");  // old style, not constantly set to zeros

//   setActuatorMaxVelocity =
//       (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetActuatorMaxVelocity");  // no corresponding DSP
//       code.

//   // %EndTag(not function, place holder, etc)%

//   // %Tag(experimental)%
//   // force/torque control are not tested in ROS and may lead to unexpect motion.
//   // do not use these functions unless you are know exactly what you are doing.

//   setTorqueControlType = (int (*)(TORQUECONTROL_TYPE))initCommandLayerFunction("SetTorqueControlType");

//   setActuatorPIDFilter = (int (*)(int, float, float, float))initCommandLayerFunction("SetActuatorPIDFilter");

//   setAngularInertiaDamping = (int (*)(AngularInfo, AngularInfo))initCommandLayerFunction("SetAngularInertiaDamping");

//   getAngularForceGravityFree = (int (*)(AngularPosition&))initCommandLayerFunction("GetAngularForceGravityFree");

//   setTorqueActuatorGain = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueActuatorGain");

//   setTorqueActuatorDamping = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueActuatorDamping");

//   setTorqueCommandMax = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueCommandMax");

//   setTorqueRateLimiter = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueRateLimiter");

//   setTorqueFeedCurrent = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFeedCurrent");

//   setTorqueFeedVelocity = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFeedVelocity");

//   setTorquePositionLimitDampingGain = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorquePositionLimitDam"
//                                                                                              "pingGain");

//   setTorquePositionLimitDampingMax = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorquePositionLimitDamp"
//                                                                                             "ingMax");

//   setTorquePositionLimitRepulsGain = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorquePositionLimitRepu"
//                                                                                             "lsGain");

//   setTorquePositionLimitRepulsMax = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorquePositionLimitRepul"
//                                                                                            "sMax");

//   setTorqueFilterVelocity = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFilterVelocity");

//   setTorqueFilterMeasuredTorque = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFilterMeasuredTorqu"
//                                                                                          "e");

//   setTorqueFilterError = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFilterError");

//   setTorqueFilterControlEffort = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetTorqueFilterControlEffort");

//   setGravityType = (int (*)(GRAVITY_TYPE))initCommandLayerFunction("SetGravityType");

//   setGravityVector = (int (*)(float[GRAVITY_VECTOR_SIZE]))initCommandLayerFunction("SetGravityVector");

//   getAngularTorqueCommand = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("GetAngularTorqueCommand");

//   getAngularTorqueGravityEstimation = (int
//   (*)(float[COMMAND_SIZE]))initCommandLayerFunction("GetAngularTorqueGravityEs"
//                                                                                              "timation");

//   setSwitchThreshold = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetSwitchThreshold");

//   setPositionLimitDistance = (int (*)(float[COMMAND_SIZE]))initCommandLayerFunction("SetPositionLimitDistance");

//   setTorqueVibrationController = (int (*)(float))initCommandLayerFunction("SetTorqueVibrationController");

//   setTorqueRobotProtection = (int (*)(int))initCommandLayerFunction("SetTorqueRobotProtection");

//   getTrajectoryTorqueMode = (int (*)(int&))initCommandLayerFunction("GetTrajectoryTorqueMode");

//   setTorqueInactivityType = (int (*)(int))initCommandLayerFunction("SetTorqueInactivityType");

//   // %EndTag(experimental)%
// }

}  // namespace armms
