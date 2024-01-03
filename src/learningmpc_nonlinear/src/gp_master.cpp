#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <learningmpc_nonlinear/track.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <learningmpc_nonlinear/gp_learn.h>
#include <ltv_mpc/gpinput.h>
#include <ltv_mpc/gpoutput.h>
#include <ltv_mpc/gp_data.h>
#include <ltv_mpc/gp_srv.h>
using namespace std;

int input_dim = 6;
string covfun1_ = "CovSEiso";
string covfun2_ = "CovNoise";
int max_num_ = 300;
GP_Learn *gp_learn_v_;
GP_Learn *gp_learn_r_;
GP_Learn *gp_learn_beta_;

GP_Learn *gp_learn_v_copy_;
GP_Learn *gp_learn_r_copy_;
GP_Learn *gp_learn_beta_copy_;
int marker_spin = 0;
int copy_use = 0;

void initializeGP(){
  gp_learn_v_ = new GP_Learn(input_dim, covfun1_, covfun2_, max_num_);
  gp_learn_r_ = new GP_Learn(input_dim, covfun1_, covfun2_, max_num_);
  gp_learn_beta_ = new GP_Learn(input_dim, covfun1_, covfun2_, max_num_);
  gp_learn_v_copy_ = new GP_Learn(*gp_learn_v_);
  gp_learn_r_copy_ = new GP_Learn(*gp_learn_r_);
  gp_learn_beta_copy_ = new GP_Learn(*gp_learn_beta_);

  Eigen::Vector3d params_v = {6.67699, 2.90749, 3.05803};
  Eigen::Vector3d params_r = {5.81545, 6.06309, 3.12376};
  Eigen::Vector3d params_beta = {5.74294, 5.56467, 2.38394};

  gp_learn_v_->SetParams(params_v);
  gp_learn_r_->SetParams(params_r);
  gp_learn_beta_->SetParams(params_beta);
  marker_spin = 1;
}

void input_callback(const ltv_mpc::gp_data::ConstPtr &data_msg){
  ltv_mpc::gpinput data_input= data_msg->gp_input;
  ltv_mpc::gpoutput data_output= data_msg->gp_output;

  double x_v_data[] = {data_input.v * 10.0, data_input.r * 10.0, data_input.beta * 100.0, data_input.Steer * 100.0, data_input.Tf * 10.0, data_input.Tr * 10.0};
  double x_r_data[] = {data_input.v, data_input.r * 100.0, data_input.beta * 1000.0, data_input.Steer * 1000.0, data_input.Tf, data_input.Tr};
  double x_beta_data[] = {data_input.v, data_input.r * 100.0, data_input.beta * 1000.0, data_input.Steer * 1000.0, data_input.Tf, data_input.Tr};

  gp_learn_v_->AddPattern(x_v_data, data_output.v_error * 10000.0);
  gp_learn_r_->AddPattern(x_r_data, data_output.r_error * 1000.0);
  gp_learn_beta_->AddPattern(x_beta_data, data_output.beta_error * 10000.0);

  marker_spin = 0;
  if (copy_use == 1){
    return;
  }else{
    marker_spin = 0;
    delete gp_learn_v_copy_;
    delete gp_learn_r_copy_;
    delete gp_learn_beta_copy_;
    gp_learn_v_copy_ = new GP_Learn(*gp_learn_v_);
    gp_learn_r_copy_ = new GP_Learn(*gp_learn_r_);
    gp_learn_beta_copy_ = new GP_Learn(*gp_learn_beta_);

    marker_spin = 1;
  }
}

bool gp_call_back(ltv_mpc::gp_srv::Request &req, ltv_mpc::gp_srv::Response &res){
  if (marker_spin == 0 || !gp_learn_v_->DataEnough()){
    return false;
  }
  double x_v[] = {req.gp_input_now.v * 10.0, req.gp_input_now.r * 10.0, req.gp_input_now.beta * 100.0, req.gp_input_now.Steer * 100.0, req.gp_input_now.Tf * 10.0, req.gp_input_now.Tr * 10.0};
  double x_r[] = {req.gp_input_now.v, req.gp_input_now.r * 100.0, req.gp_input_now.beta * 1000.0, req.gp_input_now.Steer * 1000.0, req.gp_input_now.Tf, req.gp_input_now.Tr};
  double x_beta[] = {req.gp_input_now.v, req.gp_input_now.r * 100.0, req.gp_input_now.beta * 1000.0, req.gp_input_now.Steer * 1000.0, req.gp_input_now.Tf, req.gp_input_now.Tr};
  copy_use = 1;
  res.gp_output_now.v_error = (gp_learn_v_copy_->CalculateMu(x_v))/10000.0;
  res.gp_output_now.r_error = (gp_learn_r_copy_->CalculateMu(x_r))/1000.0;
  res.gp_output_now.beta_error = (gp_learn_beta_copy_->CalculateMu(x_beta))/10000.0;
  copy_use = 0;
  return true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "gp_master");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/gp_data_topic", 1, input_callback);
  ros::ServiceServer gp_service = nh.advertiseService("gp_server", gp_call_back);
  //cout << "here2"<<endl;
  ros::Duration(0.5).sleep();


  //初始化NMPC控制器
  initializeGP();
  ROS_INFO("GP online");


  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  delete gp_learn_v_copy_;
  delete gp_learn_r_copy_;
  delete gp_learn_beta_copy_;

  delete gp_learn_v_;
  delete gp_learn_r_;
  delete gp_learn_beta_;
}
