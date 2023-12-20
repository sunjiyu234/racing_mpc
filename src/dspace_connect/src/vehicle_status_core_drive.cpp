#include <dspace_connect/vehicle_status_core.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fstream>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

struct Position {
    double x0;
    double y0;
    double yaw;
};
struct Vehicle_state{
    double vx;
    double vy;
    double yaw_rate;
    double beta;
};
struct Steer{
    double steer_l;
    double steer_r;
    double steer;
};
struct Torque{
    double Tf;
    double Tr;
};
struct Accel{
    double ax;
    double ay;
};

namespace VehicleStatusNS
{

  VehicleStatus::VehicleStatus()
  {
     pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
     sub_desired_status = nh.subscribe("nav", 1, &VehicleStatus::CallbackGetDesiredStatus, this);
     can_state_pub = nh.advertise< sensor_msgs::Imu>("state_odom", 10);

    ros::NodeHandle pnh("~");
    pnh.param<int>("device_type", device_type, 4);  //设备类型
    pnh.param<int>("device_index", device_index, 0);  //设备号
    pnh.param<int>("channel_index", channel_index, 0);  //通道号
    pnh.param<int>("baund_rate", baund_rate, 500);  //波特率
    pnh.param<bool>("remote_flag", remote_flag, false);  //是否为远程帧
    pnh.param<bool>("extern_flag", extern_flag, false);  //是否为扩展帧

  }

  VehicleStatus::~VehicleStatus()
  {
  }

  void VehicleStatus::DateProcessAndSend(PVCI_CAN_OBJ date, DWORD length)
  {
    Position p_now;
    Vehicle_state state_now;
    Steer steer_now;
    Accel accel_now;
    Torque torque_now;
    std::ofstream ofs;
    auto now_time = std::chrono::system_clock::now( );
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
    auto time_value = now_ms.time_since_epoch().count();
    ros::Time timestamp = ros::Time::now();
    nav_msgs::Odometry can_status;
    sensor_msgs::Imu can_state_status;
    for (int i = 0; i < length; i++)
    {
      ROS_INFO("current i = %d", i);
      ROS_INFO("current ID = %d", date[i].ID);
      switch (date[i].ID)
      {
      case 0x245: //速度角度信号接收
        p_now.x0 = (date[i].Data[0] + date[i].Data[1] * 16 * 16) * 0.02 - 655;
        p_now.y0 = (date[i].Data[2] + date[i].Data[3] * 16 * 16) * 0.02 - 655;
        p_now.yaw = (date[i].Data[4] + date[i].Data[5] * 16 * 16) * 0.0001 - 3.2;
        now_time = std::chrono::system_clock::now( );
        now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
        time_value = now_ms.time_since_epoch().count();
        ROS_INFO("Now is 0x245! x0 = %lf, y0 = %lf, yaw = %lf, time = %d\n", p_now.x0, p_now.y0, p_now.yaw, time_value);
        break;
      case 0x345:
        state_now.vx = (date[i].Data[0] + date[i].Data[1] * 16 * 16) * 0.0025 - 20;
        now_time = std::chrono::system_clock::now( );
        now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
        time_value = now_ms.time_since_epoch().count();
        ROS_INFO("Now is 0x345! vx = %lf, time = %d\n", state_now.vx, time_value);
        break;
      case 0x145:
        state_now.yaw_rate = (date[i].Data[0] + date[i].Data[1] * 16 * 16) * 0.0002 - 6.28;
        state_now.vy = (date[i].Data[2] + date[i].Data[3] * 16 * 16) * 0.0003 - 9.8;
        state_now.beta = (date[i].Data[4] + date[i].Data[5] * 16 * 16) * 0.0001 - 3.2767;
        now_time = std::chrono::system_clock::now( );
        now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
        time_value = now_ms.time_since_epoch().count();
        ROS_INFO("Now is 0x145! yaw_rate = %lf, vy = %lf, beta = %lf,  time = %d\n", state_now.yaw_rate, state_now.vy, state_now.beta,  time_value);
        break;
      case 0x2FF:
        steer_now.steer_l = (date[i].Data[0] + date[i].Data[1] * 16 * 16) * 0.001526 - 50.0;
        steer_now.steer_r = (date[i].Data[2] + date[i].Data[3] * 16 * 16) * 0.001526 - 50.0;
        steer_now.steer = (steer_now.steer_l + steer_now.steer_r) * M_PI / 180.0/ 2.0;
        now_time = std::chrono::system_clock::now( );
        now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
        time_value = now_ms.time_since_epoch().count();
        ROS_INFO("Now is 0x2FF! steer_l = %lf, steer_r = %lf, time = %d\n", steer_now.steer_l, steer_now.steer_r, time_value);
        break;
      case 0x3FF:
        torque_now.Tf = (date[i].Data[0] + date[i].Data[1] * 16 * 16) * 0.08 - 2500.0;
        torque_now.Tr = (date[i].Data[2] + date[i].Data[3] * 16 * 16) * 0.08 - 2500.0;
        accel_now.ax = (date[i].Data[4] + date[i].Data[5] * 16 * 16) * 0.00025 - 8.0;
        accel_now.ay = (date[i].Data[6] + date[i].Data[7] * 16 * 16) * 0.0005 - 16.0;
        now_time = std::chrono::system_clock::now( );
        now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
        time_value = now_ms.time_since_epoch().count();
        ROS_INFO("Now is 0x3FF! Tf = %lf, Tr = %lf, ax = %lf, ay = %lf, time = %d\n", torque_now.Tf, torque_now.Tr, accel_now.ax, accel_now.ay, time_value);
        // ROS_INFO("state! yaw_rate = %lf, vy = %lf, beta = %lf,  time = %d\n", state_now.yaw_rate, state_now.vy, state_now.beta,  time_value);
        timestamp = ros::Time::now();
        can_status.header.stamp = timestamp;
        can_status.header.frame_id = "map";
        can_status.child_frame_id = "base_link";
        can_status.pose.pose.position.x = p_now.x0;
        can_status.pose.pose.position.y = p_now.y0;
        can_status.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p_now.yaw);
        can_status.twist.twist.linear.x = state_now.vx;
        can_status.twist.twist.linear.y = state_now.vy;
        can_status.twist.twist.angular.z = state_now.yaw_rate;
        can_state_status.angular_velocity.x = torque_now.Tf;
        can_state_status.angular_velocity.y = torque_now.Tr;
        can_state_status.angular_velocity.z = steer_now.steer;
        can_state_status.linear_acceleration.x = accel_now.ax;
        can_state_status.linear_acceleration.y = accel_now.ay;
        pub_odom.publish(can_status);
        can_state_pub.publish(can_state_status);
        break;
      default:
        ROS_INFO("unkonown can id");
        std::cout << "id = "<< date[i].ID<< std::endl;
        break;
      }
    }
    ofs.close();
  }

  bool VehicleStatus::OpenCAN()  //返回布尔值
  {
    if (VCI_OpenDevice(device_type, device_index, 0) != 1)
    {
      std::cout << "open device failed!" << std::endl;
      return false;
    }

    VCI_INIT_CONFIG vic;
    vic.AccCode = 0x80000008;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 1;
    switch (baund_rate)
    {
    case 10:
      vic.Timing0 = 0x31;
      vic.Timing1 = 0x1c;
      break;
    case 20:
      vic.Timing0 = 0x18;
      vic.Timing1 = 0x1c;
      break;
    case 40:
      vic.Timing0 = 0x87;
      vic.Timing1 = 0xff;
      break;
    case 50:
      vic.Timing0 = 0x09;
      vic.Timing1 = 0x1c;
      break;
    case 80:
      vic.Timing0 = 0x83;
      vic.Timing1 = 0xff;
      break;
    case 100:
      vic.Timing0 = 0x04;
      vic.Timing1 = 0x1c;
      break;
    case 125:
      vic.Timing0 = 0x03;
      vic.Timing1 = 0x1c;
      break;
    case 200:
      vic.Timing0 = 0x81;
      vic.Timing1 = 0xfa;
      break;
    case 250:
      vic.Timing0 = 0x01;
      vic.Timing1 = 0x1c;
      break;
    case 400:
      vic.Timing0 = 0x80;
      vic.Timing1 = 0xfa;
      break;
    case 500:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x1c;
      break;
    case 666:
      vic.Timing0 = 0x80;
      vic.Timing1 = 0xb6;
      break;
    case 800:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x16;
      break;
    case 1000:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x14;
      break;
    case 33:
      vic.Timing0 = 0x09;
      vic.Timing1 = 0x6f;
      break;
    case 66:
      vic.Timing0 = 0x04;
      vic.Timing1 = 0x6f;
      break;
    case 83:
      vic.Timing0 = 0x03;
      vic.Timing1 = 0x6f;
      break;
    default:
      break;
    }
    vic.Mode = 0;
    if (VCI_InitCAN(device_type, device_index, 0, &vic) != 1 || VCI_InitCAN(device_type, device_index, 1, &vic) != 1)
    {
      std::cout << "init can failed!" << std::endl;
      return false;
    }

    if (VCI_ClearBuffer(device_type, 0, 0) != 1 || VCI_ClearBuffer(device_type, 0, 1) != 1)
    {
      std::cout << "clear buffer failed!" << std::endl;
      return false;
    }

    if (VCI_StartCAN(device_type, 0, 0) != 1 || VCI_StartCAN(device_type, 0, 1) != 1)
    {
      std::cout << "start can failed!" << std::endl;
      return false;
    }

    return true;
  }

  void VehicleStatus::CloseCAN()
  {
    VCI_CloseDevice(device_type, device_index);
  }

  DWORD VehicleStatus::ReceiveDate(PVCI_CAN_OBJ date, int size)
  {
    if (VCI_GetReceiveNum(device_type, device_index, channel_index) <= 0)
    {
      return 0;
    }

    return VCI_Receive(device_type, device_index, channel_index, date, size, 0);
  }
  void VehicleStatus::SendDate(PVCI_CAN_OBJ date, DWORD length)
  {
    VCI_Transmit(device_type, device_index, channel_index, date, length);
  }

  void VehicleStatus::CallbackGetDesiredStatus(const ackermann_msgs::AckermannDriveStampedConstPtr &msgs)
  {
    auto now_time_start = std::chrono::system_clock::now( );
    auto now_ms_start = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time_start);
    auto time_value_start = now_ms_start.time_since_epoch().count();
    double steering_ratio = 1.0;
    double wheel_steer = msgs->drive.steering_angle;
    VCI_CAN_OBJ date[2];
    DWORD length = 2;
    date[0].ID =  0x100; //转角发送
    date[0].RemoteFlag = remote_flag;
    date[0].ExternFlag = extern_flag;
    date[0].DataLen = 8;
    date[0].Data[0] =  (int)((wheel_steer  + 540) / 0.01648) % (16 * 16);   // -1->-439, 1->439, min -540, max 540
    date[0].Data[1] =  (int)((wheel_steer + 540) / 0.01648) / (16 * 16);
    date[0].Data[2] = (int)((wheel_steer  + 50) / 0.001526) % (16 * 16);
    date[0].Data[3] = (int)((wheel_steer  + 50) / 0.001526) /  (16 * 16);
    date[0].Data[4] = (int)((wheel_steer  + 50) / 0.001526) % (16 * 16);
    date[0].Data[5] = (int)((wheel_steer  + 50) / 0.001526) /  (16 * 16);
    date[0].Data[6] = 0x00;
    date[0].Data[7] = 0x00;
    auto now_time_1 = std::chrono::system_clock::now( );
    auto now_ms_1 = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time_1);
    auto time_value_1 = now_ms_1.time_since_epoch().count();
   double accel = msgs->drive.acceleration;
   double speed = msgs->drive.speed;
    date[1].ID = 0x1FF; //油门发送
    date[1].RemoteFlag = remote_flag;
    date[1].ExternFlag = extern_flag;
    date[1].DataLen = 8;
    date[1].Data[0] = (int)((accel + 8) / 0.00025) % (16 * 16);   // -100-100, min -120, max 120
    date[1].Data[1] = (int)((accel + 8) / 0.00025) / (16 * 16);
    date[1].Data[2] = (int)((speed * 3.6 + 20) / 0.0025) % (16 * 16);
    date[1].Data[3] = (int)((speed * 3.6 + 20) / 0.0025) / (16 * 16);
    date[1].Data[4] = (int)((speed + 2500) / 0.08) % (16 * 16);
    date[1].Data[5] = (int)((speed + 2500) / 0.08) / (16 * 16);
    date[1].Data[6] = (int)((accel + 2500) / 0.08) % (16 * 16);
    date[1].Data[7] = (int)((accel + 2500) / 0.08) / (16 * 16);
    auto now_time_2 = std::chrono::system_clock::now( );
    auto now_ms_2 = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time_1);
    auto time_value_2 = now_ms_2.time_since_epoch().count();
    SendDate(date, length);
    auto now_time = std::chrono::system_clock::now( );
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
    auto time_value = now_ms.time_since_epoch().count();
    fout << "time_start = "<< time_value_start << ", "<< "time_1 = "<< time_value_1 << ", "<< "time_2 = "<< time_value_2 << ", "<<"time = " << time_value << ", " << "accel = " <<  accel << "speed = "<< speed << ", " << "steer_angle = "<< wheel_steer << std::endl;
  }

  void VehicleStatus::stop()
  {
    VCI_CAN_OBJ date[2];
    DWORD length = 2;
    date[0].ID =  0x100; //转角报文发送
    date[0].RemoteFlag = remote_flag;
    date[0].ExternFlag = extern_flag;
    date[0].DataLen = 8;
    date[0].Data[0] = (int)((0 + 540) / 0.01648) % (16 * 16);
    date[0].Data[1] = (int)((0 + 540) / 0.01648) % (16 * 16);
    date[0].Data[2] = (int)((0 + 50) / 0.001526) % (16 * 16);
    date[0].Data[3] = (int)((0 + 50) / 0.001526) % (16 * 16);
    date[0].Data[4] = (int)((0 + 50) / 0.001526) % (16 * 16);
    date[0].Data[5] = (int)((0 + 50) / 0.001526) % (16 * 16);
    date[0].Data[6] = 0x00;
    date[0].Data[7] = 0x00;
  
    date[1].ID = 0x1FF; //油门转角发送
    date[1].RemoteFlag = remote_flag;
    date[1].ExternFlag = extern_flag;
    date[1].DataLen = 8;
    date[1].Data[0] = (int)((0 + 8) / 0.00025) % (16 * 16);   // -100-100, min -120, max 120
    date[1].Data[1] = (int)((0 + 8) / 0.00025) / (16 * 16);
    date[1].Data[2] = (int)((0 + 20) / 0.0025) % (16 * 16);
    date[1].Data[3] = (int)((0 + 20) / 0.0025) / (16 * 16);
    date[1].Data[4] = (int)((0+ 2500) / 0.08) % (16 * 16);
    date[1].Data[5] = (int)((0 + 2500) / 0.08) / (16 * 16);
    date[1].Data[6] = (int)((0 + 2500) / 0.08) % (16 * 16);
    date[1].Data[7] = (int)((0 + 2500) / 0.08) / (16 * 16);
    SendDate(date, length);
  }

  void VehicleStatus::MainLoop()
  {

    ros::Duration(3).sleep();
    if (!OpenCAN())
    {
      return;
    }

    ros::Rate rate_loop(20);
    fout.open("/home/sun234/racing_work/src/dspace_connect/data/controller_send_data");
    while (ros::ok())
    {
      ros::spinOnce();

      VCI_CAN_OBJ date[2500];
      DWORD length = ReceiveDate(date, 2500);
      if (length > 0)
      {
        // ROS_INFO("LENGTH = %d\n", length );
        DateProcessAndSend(date, length);
      }
      rate_loop.sleep();
    }

    stop();

    ros::Duration(0.5).sleep();

    CloseCAN();
    fout.close();
  }
}
