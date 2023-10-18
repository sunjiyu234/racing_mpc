#include "controller_mpc/lms_filter.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tuple>
#include<iostream>

lms_filter::lms_filter(Eigen::Matrix<double, 2, 2> input_miu)
{
    miu = input_miu;
    E << 1.0, 0.0, 0.0, 1.0;
    pre_x << 0.0, 0.0;
    pre_dt << 0.0, 0.0;
}
void lms_filter::set_model(double pre_vx){
    A << -(C_f +C_r)/(pre_vx * m)*dT + 1.0, (-pre_vx - (l_f*C_f - l_r*C_r)/(pre_vx * m))*dT, -(l_f*C_f - l_r*C_r)/(pre_vx * Iz)*dT, -(l_f*l_f*C_f + l_r*l_r*C_r)/(pre_vx*Iz)*dT + 1.0;
    B << C_f*dT/m, l_f*C_f*dT/Iz;
}
void lms_filter::calculate_next_state(double pre_control){
    x_t_per = A*pre_x + B * pre_control+E * pre_dt;
}

std::tuple<double, double> lms_filter::solution_ans(Eigen::Vector2d xt, double pre_control){
    et = xt - x_t_per;
    dt = pre_dt + miu *E*et;
    pre_x = A*pre_x + B*pre_control+E*dt;
    pre_dt = dt;
    return std::make_tuple(pre_x(0), pre_x(1));
}
