#ifndef LMS_FILTER_H
#define LMS_FILTER_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tuple>


class lms_filter
{
public:
    lms_filter(Eigen::Matrix<double, 2, 2> input_miu);
    void set_model(double pre_vx);
    void calculate_next_state(double pre_control);
    std::tuple<double, double> solution_ans(Eigen::Vector2d xt, double pre_control);
private:
    Eigen::Matrix<double, 2, 2> miu;
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 2> E; // I
    Eigen::Vector2d et;  //error
    Eigen::Vector2d pre_x;
    Eigen::Vector2d x_t_per;
    double pre_u;
    Eigen::Vector2d pre_dt;
    Eigen::Vector2d dt; //offset

    //para
    double C_f = 78939;
    double C_r = 44493;
    double l_f = 1.015;
    double l_r = 1.895;
    double m = 1270;
    double Iz = 1536.7;
    double l = 2.91;
    double h = 0.54;
    double dT = 0.02;
};

#endif // LMS_FILTER_H
