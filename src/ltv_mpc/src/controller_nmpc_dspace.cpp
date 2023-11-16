#include <ros/ros.h>
#include "controller_mpc/path_planner.h"
#include "controller_mpc/nmpc_dspace.h"
#include "controller_mpc/pid_speed.h"
#include "controller_mpc/lms_filter.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <cmath>
#include <chrono>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>
#include <cstring>
#include <stdlib.h> //stand library标准库
#include <sstream>  //字符串转换
#include <vector> //向量头文件
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fstream> //提供文件头文件
#include <iomanip> //C++输出精度控制需要
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tuple>
using namespace std;

double v_target = 10.0;  //目标车速
double gCurvatureK = 150;
int marker_spin = 0;
double cutoff_freq = 100;
double RC = 1.0 /(cutoff_freq * 2.0 * M_PI);
double alpha = 0.02 /(0.02 + RC);

struct vehicleState{
  double x;
  double y;
  double vx;
  double vy;
  double yaw;
  double r;
};   //结构体用于储存组合导航得到的实时车辆数据
vehicleState gState;
PathPlanner::Planner gPlanner;
NMPC::IterativeController gController;  //内含两个全局函数
double kp = 5.0;
double ki = 0.0;
double kd = 0.0;
pid_speed pid_controller(kp, ki, kd);

//用于存储底盘返回的角度和速度
double angle_back = 0.0;
double speed_back = 0.0;

//初始化NMPC
void initializeNMPC(){
  NMPC::Parameters parameters;
  parameters.dt = 0.05;
  parameters.pred_horizon = 20;
  parameters.control_horizon = 10;

  NMPC::Model model;
  model.l_f = 1.165;   //质心前轴距
  model.l_r = 1.165;   //质心后轴距
  model.m = 1140.0;       //车辆质量
  model.I_z = 2849.5;        //车辆横摆惯量*
  model.l = 2.33;      //车辆轴距
  model.h = 0.305;         //车辆质心高*
  model.g = 9.8;
  model.vx = v_target;
  //车辆轮胎魔术模型参数*
  model.B_f = 12.6167;
  model.C_f = 1.3;
  model.D_f = 4748;
  model.E_f = -1.5;
  model.B_r = 13.7677;
  model.C_r = 1.3;
  model.D_r = 4748;
  model.E_r = -1.5;

  NMPC::HardConstraint constraint;
  constraint.max_steer = 30.0 / 180 * M_PI;
  constraint.max_steer_rate = 100.0 / 180 * M_PI;
  constraint.max_ey = 0.05;
  constraint.max_eyaw = 5.0/180.0*M_PI;

  NMPC::CostFunctionWeights weights;
  weights.w_r = 0.0;
  weights.w_v_y = 0.5;
  weights.w_e_yaw =  1.0;
  //weights.w_e_d = 0.75;
  weights.w_e_d = 2.0;
  weights.w_steer_angle = 5000.0;
  //weights.w_steer_angle =15.0;
  //weights.w_steer_angle =20.0;
  //weights.w_steer_angle =50.0;
  weights.w_slack_eyaw = 500.0;
  weights.w_slack_ey = 500.0;

  gController.initialize(parameters, model, constraint, weights);    //gController为全局
}

//对接收的GPS信息进行处理
/*
void state_callback(const std_msgs::String::ConstPtr& msgs)
{
  //cout << "call back" <<endl;
  string CAN_data = msgs -> data;
  istringstream CAN_line(CAN_data); //新建信息流
  vector <string> CAN_msgs;  //建立容器收集经纬度，速度，横摆角等信息
  string CAN_word;

  while(getline(CAN_line,CAN_word,','))  //按照逗号分隔字符串
  {
    CAN_msgs.push_back(CAN_word);
  }
  //cout << GPS_msgs <<endl;
  string CAN_x0 = CAN_msgs[1];
  string CAN_y0 = CAN_msgs[2];
  string CAN_yaw = CAN_msgs[3];
  string CAN_vx = CAN_msgs[4];
  string CAN_vy = CAN_msgs[5];
  string CAN_yaw_rate = CAN_msgs[6];
  string CAN_steer_l = CAN_msgs[7];
  string CAN_steer_r = CAN_msgs[8];

  stringstream s_can_x, s_can_y, s_can_yaw, s_can_v, s_can_vy, s_can_r, s_can_steer_l, s_can_steer_r;
  s_can_x << CAN_x0;
  s_can_y << CAN_y0;
  s_can_yaw << CAN_yaw;
  s_can_v << CAN_vx;
  s_can_vy << CAN_vy;
  s_can_r << CAN_yaw_rate;
  s_can_steer_l << CAN_steer_l;
  s_can_steer_r << CAN_steer_r;

  s_can_x >> gState.x;
  s_can_y >> gState.y;
  s_can_yaw >> gState.yaw;
  s_can_v >> gState.vx;
  s_can_vy >> gState.vy;
  s_can_r >> gState.r;
  marker_spin = 1;
}
*/
void state_callback(const nav_msgs::Odometry::ConstPtr& msgs)
{
  //cout << "call back" <<endl;
  nav_msgs::Odometry CAN_data = *msgs;
  gState.x = CAN_data.pose.pose.position.x;
  gState.y = CAN_data.pose.pose.position.y;
  gState.yaw =  tf::getYaw(CAN_data.pose.pose.orientation);
  gState.vx = CAN_data.twist.twist.linear.x;
  gState.vy = CAN_data.twist.twist.linear.y;
  gState.r = CAN_data.twist.twist.angular.z;
  marker_spin = 1;
  auto now_time = std::chrono::system_clock::now( );
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now_time);
  auto time_value = now_ms.time_since_epoch().count();
  ROS_INFO("GET state! time = %d\n", time_value);
}
void angle_call_back(const geometry_msgs::TwistStamped& msgs){
  angle_back = msgs.twist.angular.x;
}
void speed_call_back(const geometry_msgs::TwistStamped& msgs){
  speed_back = msgs.twist.linear.x;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_nmpc");
  ros::NodeHandle nh;
  // cout << "here" <<endl;
  //发送底盘控制数据topic
  // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/desired_status",10);
  ros::Publisher cmd_vel_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 10);
  //发送跟踪的预测时域内目标轨迹
  ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("planned_trajectory", 10);
  //发送预测轨迹
  ros::Publisher  prediction_pub  = nh.advertise<geometry_msgs::PoseArray>("predicted_trajectory", 10);
  //发送参考路径
  ros::Publisher  road_pub  =  nh.advertise<nav_msgs::Path>("/ref_road",10);
  //发送当前状态记录
  ros::Publisher  state_pub   = nh.advertise<std_msgs::String>("/vehicle_state", 10);
  //接收组合导航信息
  ros::Subscriber sub = nh.subscribe("/odom", 10, state_callback);
  //ros::Subscriber sub = nh.subscribe("/can_state", 10, state_callback);
  //接受底盘反馈信息
  ros::Subscriber sub_steer = nh.subscribe("/vehicle_angle", 10, angle_call_back);
  ros::Subscriber sub_speed = nh.subscribe("/vehicle_speed", 10, speed_call_back);
  //发送实际路径
  ros::Publisher  real_pub = nh.advertise<nav_msgs::Path>("/real_road",10);
  //cout << "here2"<<endl;
  ros::Duration(0.5).sleep();

  //加载点集路径
  ros::NodeHandle nh_priv("~");
  std::string track_filename = nh_priv.param<std::string>("track_filename", "/home/sun234/racing_work/src/learningmpc_nonlinear/data/30m_circle_right.csv");
  gPlanner.cvx.push_back(v_target);
  // cout << "here3"<< endl;
  gPlanner.loadPath(track_filename,1);   //txt文件中的点存储在gPlanner.waypoints_里
  // cout << "here4"<< endl;

  //初始化NMPC控制器
  initializeNMPC();
  ROS_INFO("NMPC control online");

  double last_steer_angle = 0.0;

  //建立参考路径path信息
  nav_msgs::Path path_ref;
  path_ref.header.stamp = ros::Time::now();
  path_ref.header.frame_id = "map";
  geometry_msgs::PoseStamped poses_ref;
  poses_ref.header.stamp = ros::Time::now();
  poses_ref.header.frame_id = "map";
  for (int i =0;i < gPlanner.cx.size();i++){
    poses_ref.pose.position.x = gPlanner.cx[i];
    poses_ref.pose.position.y = gPlanner.cy[i];
    path_ref.poses.push_back(poses_ref);
//          cout << path_ref <<endl;
  }

  //建立实际path的轨迹msgs法
  nav_msgs::Path path_real;
  nav_msgs::Odometry path_real_odo;
  path_real.header.stamp = ros::Time::now();
  path_real.header.frame_id = "map";
  geometry_msgs::PoseStamped poses_real;
  poses_real.header.stamp = ros::Time::now();
  poses_real.header.frame_id ="map";

  //初始值赋予
  double x = 0.0;
  double y = 0.0;
  double yaw = M_PI/2.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
  double steer = 0.0;

  double num_error = 0;
  double pre_vx_main = v_target;
  double pre_control_main = 0.0;

  Eigen::Matrix<double, 2, 2> miu_main;
  miu_main << 0.008, 0, 0, 0.008;
  lms_filter state_filter(miu_main);

  // disturb
  const double mean = 0.0;
  //const double stddev = 0.2;
  //default_random_engine generator;
  //normal_distribution<double> dist(mean, stddev);

  ros::Rate rate(20);
  int marker = 0;
  while(ros::ok()){
    double clock_start_time = clock();
    ros::spinOnce();
    if(marker_spin == 0){
        continue;
    }
    NMPC::State state;
    if(abs(gState.vx - 0.0) < 0.001){
        cout << "now vx is 0" <<endl;
        gState.vx += 0.001;
    }
    //实际状态发布
    poses_real.pose.position.x = gState.x;
    poses_real.pose.position.y = gState.y;
    path_real.poses.push_back(poses_real);
    real_pub.publish(path_real);

    //use LMS filter to estimate wr vy
    //state_filter.set_model(pre_vx_main);
    //cout << "here12"<<endl;
    //state_filter.calculate_next_state(pre_control_main);
    //Eigen::Vector2d x_t_main;
    //x_t_main << gState.vy, gState.r;
    stringstream ss_pre_vy, ss_pre_wr, ss_get_vy, ss_get_wr;
    ss_pre_vy << gState.vy;
    ss_pre_wr << gState.r;
    //gState.r = gState.r + dist(generator);
    //gState.vy = gState.vy + dist(generator);
    ss_get_vy << gState.vy;
    ss_get_wr << gState.r;
    //cout << "here23"<<endl;

    //std::tuple<double, double> ans_filer = state_filter.solution_ans(x_t_main, pre_control_main);
    //cout << "here34"<<endl;
    //gState.vy = std::get<0>(ans_filer);
    //gState.r = std::get<1>(ans_filer);

    //状态量和控制量赋值
    state.yaw_rate = gState.r;
    state.v_y = gState.vy ;
    state.steer_angle = last_steer_angle;
    state.v_x = gState.vx;

    //plan track
    PathPlanner::PoseStamped start_pose;
    std::vector<PathPlanner::PoseStamped> track;

    start_pose.x = gState.x;
    start_pose.y = gState.y;
    start_pose.yaw = gState.yaw;
    start_pose.v = gState.vx;
    start_pose.t = 0;
    //cout <<" now state is !" <<gState.x << " " << gState.y << " " << gState.yaw << " " << gState.vx << " " << gState.vy << " " << gState.r << endl;
    double clock_get_path_start = clock();
    // cout << "here6"<< endl;
    gPlanner.getPath(start_pose, gController.parameters_.dt, v_target,0.1, gCurvatureK, 0.2, gController.parameters_.pred_horizon - 1, &track);
    state.e_yaw = gState.yaw - track[0].yaw;
    // cout << "here7"<< endl;
    //state.e_yaw = 1.74 - track[0].yaw;
    if (state.e_yaw > M_PI){
      state.e_yaw = state.e_yaw - 2.0 * M_PI;
    }
    if (state.e_yaw < -M_PI){
      state.e_yaw = state.e_yaw + 2.0 * M_PI;
    }
    state.e_d = gPlanner.error_track;
    double clock_get_path_end = clock();
    //cout << "the state is = "<<state.yaw_rate << " " << state.v_y << " "<< state.e_yaw <<" "<< state.e_d <<endl;
/*
    //显示路径曲率
    for(int i = 0;i< gController.parameters_.pred_horizon-1;i++){
        cout << "cur"<<gPlanner.curvature_cur[i]<<endl;
      }
*/
    //calculate nmpc
    NMPC::ControlOutput control_output;   //存放steer
    std::vector<NMPC::State> prediction_output;
    double clock_update_start = clock();
    double speed_error = track[0].v - gState.vx;
    double control_a;
    control_a = pid_controller.pid_control(speed_error);
    double control_a_constraint;
    control_a_constraint = max(min(6.0, control_a), -6.0);
    double control_T;
    control_T = max(min(control_a_constraint * gController.model_.m / 2.0, 2500.0), -2500.0);
    //cout <<" speed_error = " << speed_error << " control_a = "<< control_a <<" control_T = " << control_T <<endl;
    // cout << "here8"<< endl;
    gController.update(state,track,3,0.01,gPlanner.curvature_cur,&control_output,&prediction_output);
    // cout << "here9"<< endl;
    double clock_update_end = clock();
    //cout << "clock = "<< (clock_update_end-clock_update_start)/CLOCKS_PER_SEC << endl;
    double constraint = 30.0/180.0*M_PI;
    //control_output.steer = alpha * control_output.steer + (1.0 - alpha) * last_steer_angle;
    //cout << "control_output = " << control_output.steer<<endl;
    if (fabs(control_output.steer) > constraint){
      //cout << "here  "<<endl;
      num_error = num_error +1;
      control_output.steer = last_steer_angle;
    }

    //filter
    //control_output.steer = 0.8*control_output.steer + 0.2 *pre_control_main;
    last_steer_angle = control_output.steer;

    //输出控制量
    // geometry_msgs::TwistStamped cmd_vel;
    // cmd_vel.twist.linear.x = control_T;
    // cmd_vel.twist.angular.x = control_output.steer * 180.0 /M_PI;
    ackermann_msgs::AckermannDriveStamped cmd_vel;
    cmd_vel.drive.acceleration = control_T / 2.0;
    cmd_vel.drive.speed = control_T / 2.0;
    cmd_vel.drive.steering_angle = control_output.steer * 180.0 / M_PI;
    cmd_vel_pub.publish(cmd_vel);  //发送控制量
    double clock_end_time = clock();

    //状态，控制量的Publish用于记录
    std_msgs::String vehicle_state;
    stringstream ss_x, ss_y, ss_v, ss_yaw, ss_r, ss_control_v, ss_control_steer,ss_error,ss_vy,ss_yaw_rate,ss_num_error,ss_v_east,ss_v_north,ss_time,ss_time_get_path,ss_time_update,ss_e_yaw, ss_T;
    string s_state, s_x, s_y, s_v, s_yaw, s_r, s_control_v, s_control_steer,s_error,s_vy,s_yaw_rate,s_num_error,s_v_east,s_v_north,s_time,s_time_get_path,s_time_update,s_e_yaw,s_T,s_pre_vy,s_pre_wr,s_get_vy, s_get_wr;
    string comma = ",";
    ss_x << gState.x;
    ss_y << gState.y;
    ss_v << gState.vx;
    ss_vy << gState.vy;
    ss_yaw << gState.yaw;
    ss_num_error << num_error;
    ss_r << gState.r;
    ss_control_v << cmd_vel.drive.speed;
    ss_control_steer << cmd_vel.drive.steering_angle;
    ss_error << gPlanner.error_track;
    ss_e_yaw << state.e_yaw;
    ss_yaw_rate << state.yaw_rate;
    ss_T << control_T;
    //ss_v_east << *p_v_east;
    //ss_v_north << *p_v_north;

    ss_time << (clock_end_time-clock_start_time)/CLOCKS_PER_SEC;
    ss_time_get_path << (clock_get_path_end-clock_get_path_start)/CLOCKS_PER_SEC;
    ss_time_update << (clock_update_end-clock_update_start)/CLOCKS_PER_SEC;

    ss_x >> s_x;
    ss_y >> s_y;
    ss_v >> s_v;
    ss_yaw >> s_yaw;
    ss_r >> s_r;
    ss_vy >> s_vy;
    ss_control_v >> s_control_v;
    ss_control_steer >> s_control_steer;
    ss_num_error >> s_num_error;
    ss_error >> s_error;
    ss_yaw_rate >> s_yaw_rate;
    ss_pre_wr >> s_pre_wr;
    ss_pre_vy >> s_pre_vy;
    ss_get_vy >> s_get_vy;
    ss_get_wr >> s_get_wr;
    //ss_v_east >> s_v_east;
    //ss_v_north >> s_v_north;

    ss_time >> s_time;
    ss_time_update >> s_time_update;
    ss_time_get_path >> s_time_get_path;
    ss_e_yaw >> s_e_yaw;
    ss_T >> s_T;
      //s_state = s_x + comma +s_y +comma+s_v+comma+s_yaw+comma+s_r+comma+s_control_v+comma+s_control_steer+comma+s_error+comma+s_vy+comma+s_yaw_rate+comma+s_num_error;
    //s_state = s_error;
    s_state = comma+s_pre_vy+comma+s_pre_wr+comma+s_get_vy + comma + s_get_wr + comma +s_r + comma +s_vy + comma+ s_v + comma+ s_yaw+comma+ s_x+comma+s_y+comma+s_control_steer+comma + s_T +comma+s_num_error+comma+s_error + comma + s_e_yaw + comma + s_time + comma + s_time_get_path + comma + s_time_update;
    vehicle_state.data = s_state;
    state_pub.publish(vehicle_state);


    //判断到终点
    // if (abs(sqrt(pow(gState.x - gPlanner.goal_x_, 2) + pow(gState.y - gPlanner.goal_y_, 2))) < 0.25)
    // {
    //   //cout << gPlanner.goal_x_ << " " <<gPlanner.goal_y_<<endl;
    //   cmd_vel.drive.acceleration = 0.0;
    //   cmd_vel.drive.speed = 0.0;
    //   cmd_vel.drive.steering_angle = 0.0 * 180.0 / M_PI;
    //   cout <<"goal"<<endl;
    //   break;
    // }




    //visualize track，可视化目标规划路径
    geometry_msgs::PoseArray planned_trajectory;
    planned_trajectory.header.frame_id = "map";

    for(int i = 0; i < track.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = track[i].x;
        pose.position.y = track[i].y;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(track[i].yaw);
        //cout << track[i].x << " "<<track[i].y<< " "<< track[i].yaw <<endl;       //cout << track[i].yaw;

        planned_trajectory.poses.push_back(pose);
     }
     trajectory_pub.publish(planned_trajectory);   //发送规划的路径（即为给定点集的多项式拟合）

     //visualize predicted track，可视化预测路径
     geometry_msgs::PoseArray predicted_trajectory;
     predicted_trajectory.header.frame_id = "map";

     for(int i = 0; i < prediction_output.size(); i++) {
         geometry_msgs::Pose pose;
         if (i == 0){
           pose.position.x = gState.x;
           pose.position.y = gState.y;
           pose.position.z = 0;
           pose.orientation = tf::createQuaternionMsgFromYaw(gState.yaw);
         }
         if (i != 0){
           pose.position.x = predicted_trajectory.poses[i-1].position.x + v_target *  gController.parameters_.dt * cos(track[i-1].yaw + prediction_output[i-1].e_yaw);
           pose.position.y = predicted_trajectory.poses[i-1].position.y + v_target *  gController.parameters_.dt * sin(track[i-1].yaw + prediction_output[i-1].e_yaw);
           pose.position.z = 0;
           double real_yaw = track[i].yaw + prediction_output[i].e_yaw;
           if (real_yaw > M_PI){
             real_yaw = real_yaw - 2*M_PI;
           }
           if (real_yaw < -M_PI){
             real_yaw = real_yaw + 2*M_PI;
           }
           pose.orientation = tf::createQuaternionMsgFromYaw(real_yaw);
         }
         //cout << prediction_output[i].yaw;

         predicted_trajectory.poses.push_back(pose);
     }
     prediction_pub.publish(predicted_trajectory);    //发送预测的路径

     //发送跟踪的路径
     road_pub.publish(path_ref);


     //更新状态量
     x = gState.x;
     y = gState.y;
     yaw = gState.yaw;
     vy = gState.vy;
     yaw_rate = gState.r;
     steer = control_output.steer;
     pre_vx_main = gState.vx;
     pre_control_main = control_output.steer;

     //sleep and spin
     rate.sleep();
  }
}
