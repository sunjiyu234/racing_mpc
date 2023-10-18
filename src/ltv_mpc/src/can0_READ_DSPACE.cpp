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
#include <fstream>
#include <sstream>
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
    ros::Publisher can_pub = nh.advertise<std_msgs::String>("/can_state", 10);
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
    while (1) {
        nbytes = read(s, &frame, sizeof(struct can_frame)); //读取报文
        if (nbytes > 0) {
            std_msgs::String can_status;
            stringstream ss_x, ss_y, ss_v, ss_yaw, ss_r, ss_vy, ss_steer_l, ss_steer_r;
            string can_string, s_x, s_y, s_v, s_yaw, s_r, s_vy, s_steer_l, s_steer_r;
            string comma = ",";
            if (frame.can_id == 0x245) {
                p_now.x0 = (frame.data[0] + frame.data[1] * 16 * 16) * 0.05 - 1630;
                p_now.y0 = (frame.data[2] + frame.data[3] * 16 * 16) * 0.05 - 1630;
                p_now.yaw = (frame.data[4] + frame.data[5] * 16 * 16) * 0.0001 - 3.2;
                //printf("Now is 0x245! x0 = %lf, y0 = %lf, yaw = %lf\n", p_now.x0, p_now.y0, p_now.yaw);
            }else if (frame.can_id == 0x345) {
                state_now.vx = (frame.data[0] + frame.data[1] * 16 * 16) * 0.002;
                //printf("Now is 0x345! vx = %lf\n", state_now.vx);
                ss_x << p_now.x0;
                ss_y << p_now.y0;
                ss_yaw << p_now.yaw;
                ss_v << state_now.vx;
                ss_vy << state_now.vy;
                ss_r << state_now.yaw_rate;
                ss_steer_l << steer_now.steer_l;
                ss_steer_r << steer_now.steer_r;

                ss_x >> s_x;
                ss_y >> s_y;
                ss_yaw >> s_yaw;
                ss_v >> s_v;
                ss_vy >> s_vy;
                ss_r >> s_r;
                ss_steer_l >> s_steer_l;
                ss_steer_r >> s_steer_r;
                can_string = comma + s_x + comma + s_y + comma + s_yaw + comma + s_v + comma + s_vy + comma + s_r + comma + s_steer_l + comma + s_steer_r;
                can_status.data = can_string;
                can_pub.publish(can_status);
            }else if (frame.can_id == 0x145) {
                state_now.yaw_rate = (frame.data[0] + frame.data[1] * 16 * 16) * 0.0002 - 6.28;
                    state_now.vy = (frame.data[2] + frame.data[3] * 16 * 16) * 0.0003 - 9.8;
                    //printf("Now is 0x145! yaw_rate = %lf, vy = %lf\n", state_now.yaw_rate, state_now.vy);
            }else if (frame.can_id == 0x2FF) {
                steer_now.steer_l = (frame.data[0] + frame.data[1] * 16 * 16) * 0.0001 - 0.6;
                steer_now.steer_r = (frame.data[2] + frame.data[3] * 16 * 16) * 0.0001 - 0.6;
                //printf("Now is 0x2FF! steer_l = %lf, steer_r = %lf\n", steer_now.steer_l, steer_now.steer_r);
            } else {
                printf("unknown can_id");
            }
        }
    }
    close(s);
    return 0;
}
