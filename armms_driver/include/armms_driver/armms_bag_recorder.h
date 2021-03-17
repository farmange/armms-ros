#ifndef ARMMS_DRIVER_ARMMS_BAG_RECORDER_H
#define ARMMS_DRIVER_ARMMS_BAG_RECORDER_H

#include <ros/ros.h>

#include "std_srvs/Empty.h"

namespace armms_driver
{
class ArmmsBagRecorder
{
public:
  ArmmsBagRecorder(const ros::NodeHandle& nh);
  void start();
  void stop();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient start_bag_rec_srv_;
  ros::ServiceClient stop_bag_rec_srv_;

  void initializeServices_();
};

}  // namespace armms_driver

#endif