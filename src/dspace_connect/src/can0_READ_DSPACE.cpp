//
// Created by shinan_wu on 2023/3/15.
//
/* 2. 报文接收程序 */
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
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <tf/transform_listener.h>
using namespace std;
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
};

/*
#define ip_cmd_set_can1_params "sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 berr-reporting on fd on"
#define ip_cmd_can1_open       "sudo ip link set up can1"
*/
int main(int argc, char **argv) {
    /*
    system(ip_cmd_set_can1_params);
    system(ip_cmd_can1_open);
    */
    ros::init(argc, argv, "canread");
    ros::NodeHandle nh;
    ros::Publisher can_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    Position p_now;
    Vehicle_state state_now;
    Steer steer_now;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfilter[1];
    s = socket(AF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can0 绑定
    int loopback = 0; // 0 表示关闭, 1 表示开启( 默认)
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    int ro = 0; // 0 表示关闭( 默认), 1 表示开启
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &ro, sizeof(ro));

    //定义接收规则，只接收表示符等于 0x11 的报文
    /*
    rfilter[0].can_id = 0x245;
    rfilter[0].can_mask = CAN_SFF_MASK;
    */
    //设置过滤规则
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    auto now_time = std::chrono::system_clock::now( );
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
    auto time_value = now_ms.time_since_epoch().count();
    while (1) {
        nbytes = read(s, &frame, sizeof(struct can_frame)); //读取报文
        if (nbytes > 0) {
            nav_msgs::Odometry can_status;
            if (frame.can_id == 0x245) {
                p_now.x0 = (frame.data[0] + frame.data[1] * 16 * 16) * 0.02 - 655;
                p_now.y0 = (frame.data[2] + frame.data[3] * 16 * 16) * 0.02 - 655;
                p_now.yaw = (frame.data[4] + frame.data[5] * 16 * 16) * 0.0001 - 3.2;
                now_time = std::chrono::system_clock::now( );
                now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
                time_value = now_ms.time_since_epoch().count();
                printf("Now is 0x245! x0 = %lf, y0 = %lf, yaw = %lf, time = %d\n", p_now.x0, p_now.y0, p_now.yaw, time_value);
            }else if (frame.can_id == 0x345) {
                state_now.vx = (frame.data[0] + frame.data[1] * 16 * 16) * 0.0025 - 20;
                now_time = std::chrono::system_clock::now( );
                now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
                time_value = now_ms.time_since_epoch().count();
                printf("Now is 0x345! vx = %lf, time = %d\n", state_now.vx, time_value);
                can_status.pose.pose.position.x = p_now.x0;
                can_status.pose.pose.position.y = p_now.y0;
                can_status.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p_now.yaw);
                can_status.twist.twist.linear.x = state_now.vx;
                can_status.twist.twist.linear.y = state_now.vy;
                can_status.twist.twist.angular.z = state_now.yaw_rate;
                can_pub.publish(can_status);
            }else if (frame.can_id == 0x145) {
                state_now.yaw_rate = (frame.data[0] + frame.data[1] * 16 * 16) * 0.0002 - 6.28;
                state_now.vy = (frame.data[2] + frame.data[3] * 16 * 16) * 0.0003 - 9.8;
                state_now.beta = (frame.data[4] + frame.data[5] * 16 * 16) * 0.0001 - 3.2767;
                now_time = std::chrono::system_clock::now( );
                now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
                time_value = now_ms.time_since_epoch().count();
                printf("Now is 0x145! yaw_rate = %lf, vy = %lf, beta = %lf,  time = %d\n", state_now.yaw_rate, state_now.vy, state_now.beta,  time_value);
            }else if (frame.can_id == 0x2FF) {
                steer_now.steer_l = (frame.data[0] + frame.data[1] * 16 * 16) * 0.001526 - 50.0;
                steer_now.steer_r = (frame.data[2] + frame.data[3] * 16 * 16) * 0.001526 - 50.0;
                now_time = std::chrono::system_clock::now( );
                now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
                time_value = now_ms.time_since_epoch().count();
                printf("Now is 0x2FF! steer_l = %lf, steer_r = %lf, time = %d\n", steer_now.steer_l, steer_now.steer_r, time_value);
            } else {
                printf("unknown can_id");
            }
        }
    }
    close(s);
    return 0;
}
