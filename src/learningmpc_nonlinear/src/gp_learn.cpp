
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

GP_Learn::GP_Learn(const GP_Learn &gp_learn){
    cov_ = gp_learn.cov_;
    gp_ = new GaussianProcess(*(gp_learn.gp_));
}

GP_Learn::~GP_Learn(){
    if(gp_ != NULL) delete gp_;
}

void GP_Learn::SetParams(const Eigen::VectorXd &params){
    assert(params.size() == gp_->covf().get_param_dim());
    gp_->covf().set_loghyper(params);
}

void GP_Learn::AddPattern(const double x[], double y){
    double time_start_replace = clock();
    if (gp_->get_sampleset_size() < max_num_){
        gp_->add_pattern(x, y);
    }else{
        gp_->replace_pattern(x, y);
    }
    double time_end_replace = clock();
    std::cout << "Time_Add_Pattern = " << (time_end_replace  - time_start_replace)/CLOCKS_PER_SEC;
}

bool GP_Learn::DataEnough(){
    if (gp_->get_sampleset_size() < 280){
        return false;
    }else{
        return true;
    }
}

double GP_Learn::CalculateMu(const double x[]){
    return gp_->f(x);
}