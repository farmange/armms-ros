#include <armms_hardware_interface/armms_hardware_interface.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armms_hardware_interface");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  nh.setCallbackQueue(&ros_queue);
  armms_hardware_interface::ArmmsHardwareInterface rhi(nh);

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  return 0;
}