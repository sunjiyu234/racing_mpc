#include <ros/ros.h>
#include <dspace_connect/vehicle_status_core.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicleStatus");  //初始化节点vehicleStatus
  ros::NodeHandle nh;

  VehicleStatusNS::VehicleStatus v;
  v.MainLoop();
  return 0;
}
