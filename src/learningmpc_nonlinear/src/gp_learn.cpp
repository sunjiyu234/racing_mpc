
#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <std_msgs/String.h>

#include <learningmpc_nonlinear/gp_learn.h>
#include <Eigen/Dense>

using namespace libgp;

GP_Learn::GP_Learn(int input_dim, std::string cov1, std::string cov2, int max_num){
    max_num_ = max_num;
    cov_ = "CovSum ( " + cov1 + ", " + cov2 + ")";
    gp_ = new GaussianProcess(input_dim, cov_);
}

void GP_Learn::SetParams(const Eigen::VectorXd &params){
    assert(params.size() == gp_->covf().get_param_dim());
    gp_->covf().set_loghyper(params);
}

void GP_Learn::AddPattern(const double x[], double y){
    if (gp_->get_sampleset_size() < max_num_){
        gp_->add_pattern(x, y);
    }else{
        gp_->replace_pattern(0, x, y);
    }
}

double GP_Learn::CalculateMu(const double x[]){
    if (gp_->get_sampleset_size() > 300){
        return gp_->f(x);
    }else{
        return 0.0;
    }
}