//
// Created by yuwei on 11/20/19.
//

#include <ros/ros.h>
#include <ros/package.h>
#include </usr/local/include/gp/gp.h>
#include </usr/local/include/gp/gp_utils.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
using namespace libgp;

class GP_Learn{
public:
    /** Empty initialization*/
    GP_Learn(int input_dim, std::string cov1, std::string cov2, int max_num);
    GP_Learn(const GP_Learn& gp_learn);

    virtual ~GP_Learn();
    
    GaussianProcess * gp_;
    std::string cov_;

    void SetParams(const Eigen::VectorXd &params);
    bool AddPattern(const double x[], double y);
    double CalculateMu(const double x[]);
    bool DataEnough();
    
private:
    int max_num_;

};