/* 报文发送程序 */
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
struct can_frame frame[2];
int spline_marker = 0;
/*
#define ip_cmd_set_can0_params "sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 berr-reporting on fd on"
#define ip_cmd_can0_open       "sudo ip link set up can0"
*/
void can_send_callback(const geometry_msgs::TwistStamped::ConstPtr &msgs){
     frame[0].data[0] = (int)((msgs->twist.angular.x + 50) / 0.001526) % (16 * 16);
     frame[0].data[1] = (int)((msgs->twist.angular.x + 50) / 0.001526) / (16 * 16);
     frame[0].data[2] = frame[0].data[0];
     frame[0].data[3] = frame[0].data[1];

     frame[1].data[0] = (int)((msgs->twist.linear.x + 3275) / 0.1) % (16 * 16);
     frame[1].data[1] = (int)((msgs->twist.linear.x + 3275) / 0.1) / (16 * 16);
     frame[1].data[2] = frame[1].data[0];
     frame[1].data[3] = frame[1].data[1];
     spline_marker = 1;
}
int main(int argc, char **argv) {
    /*
    system(ip_cmd_set_can0_params);
    system(ip_cmd_can0_open);
    */
    ros::init(argc, argv, "cansend");
    ros::NodeHandle nh;

    ros::Subscriber sub_can = nh.subscribe("/desired_status", 10, can_send_callback);
    ros::Duration(0.5).sleep();
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    s = socket(AF_CAN, SOCK_RAW, CAN_RAW); // 创建套接字
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); // 指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); // 将套接字与 can0 绑定
   /*
    // 禁用过滤规则，本进程不接收报文，只负责发送
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    */
    int loopback = 0; // 0 表示关闭, 1 表示开启( 默认)
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    int ro = 0; // 0 表示关闭( 默认), 1 表示开启
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &ro, sizeof(ro));

    // 生成多条报文并发送
    ros::Rate rate_loop(50);
    while (ros::ok()) {
        ros::spinOnce();
        if(spline_marker == 0){
            continue;
        }
        frame[0].can_id = 0x100;
        frame[0].can_dlc = 8;
        frame[0].data[4] = 0X00;
        frame[0].data[5] = 0X00;
        frame[0].data[6] = 0X00;
        frame[0].data[7] = 0X00;
        nbytes = write(s, &frame[0], sizeof(frame[0])); // 发送报文
        if (nbytes != sizeof(frame[0])) {
            printf("Send Error!\n");
            break; // 发送错误，退出
        }
        //cout << "Sent frame " << frame[0].can_id<<endl;

        frame[1].can_id = 0x1FF;
        frame[1].can_dlc = 8;
        frame[1].data[4] = 0X00;
        frame[1].data[5] = 0X00;
        frame[1].data[6] = 0X00;
        frame[1].data[7] = 0X00;
        nbytes = write(s, &frame[1], sizeof(frame[1])); // 发送报文
        if (nbytes != sizeof(frame[1])) {
            printf("Send Error!\n");
            break; // 发送错误，退出
        }
        //cout << "Sent frame " << frame[1].can_id<<endl;

        rate_loop.sleep(); // 休眠 1 秒
    }

    close(s);
    return 0;
}
//
// Created by shinan_wu on 2023/3/15.
//
