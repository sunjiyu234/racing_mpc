#ifndef VEHICLESTATUS_CORE_H
#define VEHICLESTATUS_CORE_H

#include  <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dspace_connect/controlcan.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fstream>

namespace VehicleStatusNS
{

class VehicleStatus
{
public:
  VehicleStatus();
  ~VehicleStatus();
  void MainLoop();
  void DateProcessAndSend(PVCI_CAN_OBJ date, DWORD length);
  void CallbackGetDesiredStatus(const ackermann_msgs::AckermannDriveStampedConstPtr &msgs);
  bool OpenCAN();
  void CloseCAN();
  DWORD ReceiveDate(PVCI_CAN_OBJ date, int size);
  void SendDate(PVCI_CAN_OBJ date, DWORD length);
  void stop();

  ros::NodeHandle nh;
  ros::Publisher pub_odom;  //车辆速度
  ros::Publisher can_state_pub;
  ros::Subscriber sub_desired_status;  //

  int device_type;//设备类型
  int device_index;//设备号
  int channel_index;//通道号
  int baund_rate;//波特率
  bool remote_flag;//是否为远程帧
  bool extern_flag;//是否为扩展帧
  std::ofstream fout;
};

}

#endif // VEHICLESTATUS_CORE_H
