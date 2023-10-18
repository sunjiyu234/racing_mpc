#include "controller_mpc/pid_speed.h"

pid_speed::pid_speed(double kp, double ki, double kd)
{
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    pre_error = 0.0;
    sum_error = 0.0;
}
double pid_speed::pid_control(double error){
    double control_T;
    control_T = pid_kp * error + pid_ki * sum_error *0.02 + pid_kd * (error - pre_error)/0.02;
    pre_error = error;
    sum_error += error;
    if(sum_error < -1.0){
        sum_error = -1.0;
    }
    if(sum_error > 1.0){
        sum_error = 1.0;
    }
    return control_T;
}
pid_speed::~pid_speed(){
}
