#include <ros/ros.h>
#include <serial/serial.h>
#include <string.h>
#include <iostream>
#include <std_msgs/String.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_gps_port_for_mpc");
  ros::NodeHandle nh;
  ros::Publisher gps_pub = nh.advertise<std_msgs::String>("/gps",10);

  // 创建一个serial类
  serial::Serial sp;
  // 创建timeout
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  // 设置要打开的串口名称
  sp.setPort("/dev/ttyUSB0");
  // 设置串口通信的波特率
  sp.setBaudrate(115200);
  // 串口设置timeout
  sp.setTimeout(to);

  try {
    //打开串口
    sp.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }

  // 判断串口是否打开成功
  if(sp.isOpen())
  {
    ROS_ERROR_STREAM("/dev/ttyUSB0 is opened.");
/*
    sp.write("$cmd,output,com0,gpfpd,0.05*ff");
    sp.write("$cmd,output,com0,gtimu,0.05*ff");
*/

    sp.write("$cmd,output,com0,gpfpd,0.025*ff");
    sp.write("$cmd,output,com0,gtimu,0.025*ff");

  }
  else {
    return -1;
  }

  //ros::Rate loop_rate(10);
  ros::Rate loop_rate(20);
  //sp.write("$cmd,output,com0,gpfpd,0.1*ff");
  while(ros::ok())
  {
    //获取缓冲区内的字节数
    size_t n = sp.available();
    if(n!=0)
    {
        std_msgs::String status;
        status.data = sp.read(n);
        string read_status = status.data;
        string write_status;
        bool marker = false;

        for ( int read_num = 0;read_num <read_status.length()-1;read_num++){
          if(read_status[read_num] == '$' && read_status[read_num+2] == 'P'&& marker == false){
           // cout << read_status[read_num]  <<endl;
            write_status.append(1,read_status[read_num]);
            marker = true;
            //cout << "start_read"<<endl;
            //cout << write_status <<endl;
            continue;
          }
          if(marker == true && (read_status[read_num]!='$' || read_status[read_num+2] == 'T')){
            write_status.append(1,read_status[read_num]);

          }
          if(marker == true && read_status[read_num] == '$' && read_status[read_num+2] == 'P'){
            //cout << "read_finish"<<endl;
            //cout << write_status <<endl;
            break;
          }
            /*if(marker == true && read_status[read_num] == '/n'){
              continue;
          }
          */
        }
        status.data = write_status;
        gps_pub.publish(status);


      //把数据发送回去
     }
    loop_rate.sleep();
  }

  // 关闭串口
  sp.close();

  return 0;
}
