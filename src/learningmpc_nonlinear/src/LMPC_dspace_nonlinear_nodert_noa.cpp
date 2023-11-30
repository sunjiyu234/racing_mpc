
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
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
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <learningmpc_nonlinear/car_params.h>
#include <learningmpc_nonlinear/gp_learn.h>

const int nx = 6;   // [x, y, phi, v, r, beta]
const int nu = 3;   // [Tf, Tr, delta]
const int terminal_nx = 6;
const int gp_nx = 6;
int gp_dim = 3;
const int RRT_INTERVAL = 0.1;
using namespace std;
using namespace Eigen;
int marker_odom = 0;
int marker_state = 0;

struct Sample{
    Matrix<double,nx,1> x;
    Matrix<double,nu,1> u;
    double s;
    int time;
    int iter;
    int cost;
};
enum rviz_id{
    CENTERLINE,
    CENTERLINE_POINTS,
    CENTERLINE_SPLINE,
    PREDICTION,
    BORDERLINES,
    SAFE_SET,
    TERMINAL_CANDIDATE,
    DEBUG
};

class LMPC{
public:
    LMPC(ros::NodeHandle& nh);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher track_viz_pub_;
    ros::Publisher LMPC_viz_pub_;
    ros::Publisher pred_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher debugger_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber rrt_sub_;
    ros::Subscriber map_sub_;

    /*Paramaters*/
    CarParams car;
    string pose_topic;
    string state_topic;
    string drive_topic;
    string waypoint_file;
    string init_data_file;
    double WAYPOINT_SPACE;
    double Ts;

    int N;
    int K_NEAR;
    double SPEED_MAX;
    double STEER_MAX;
    double ACCELERATION_MAX;
    double DECELERATION_MAX;
    double TF_LAST;
    double TR_LAST;
    double STEER_LAST;
    double DERT_TF_MAX;
    double DERT_TR_MAX;
    double DERT_STEER_MAX;
    double TF_MIN;
    double TF_MAX;
    double TR_MIN;
    double TR_MAX;
    double MAP_MARGIN;
    double VEL_THRESHOLD;
    // MPC params
    double q_s;
    double q_s_terminal;
    double q_time;
    double q_v;
    double r_Tf;
    double r_Tr;
    double r_steer;
    double vx_error_;
    Matrix<double, nu, nu> R;

    Track* track_;
    //odometry
    tf::Transform tf_;
    tf::Vector3 car_pos_;
    double yaw_;
    double x_;
    double y_;
    double vel_;
    double yawdot_;
    double slip_angle_;
    double last_vel_;
    double last_yawdot_;
    double last_slip_angle_;
    double error_v_input_;
    double error_r_input_;
    double error_beta_input_;
    double correct_v_0;
    double correct_r_0;
    double correct_beta_0;
    bool isi0_;
    bool start_record_ = false;
    int run_num_;
    double s_prev_;
    double s_curr_;
    double s_ref_cur_;
    double phi_;
    double Tf_;
    double Tr_;
    double delta_;
    double ax_;
    double ay_;
    double vx_;
    double vy_;
    ofstream lap_time_file_;
    int lap_;
    bool status_;
    int error_num_;

    string covfun1_;
    string covfun2_;
    int input_dim_;
    Vector3d params_;
    double ell_;
    double sf_;
    double sn_;
    int max_num_;

    double x_input_[gp_nx];
    double x_now_input_[gp_nx];

    GP_Learn *gp_learn_v_;
    GP_Learn *gp_learn_r_;
    GP_Learn *gp_learn_beta_;

    Matrix<double, nx, 1> add_state_pred_;

    // use dynamic model or not
    bool use_dyn_;

    //Sample Safe set
    vector<vector<Sample>> SS_;
    vector<Sample> curr_trajectory_;
    int iter_;
    int time_;
    Matrix<double,nx,1> terminal_state_pred_;

    // map info
    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid map_updated_;
    vector<geometry_msgs::Point> rrt_path_;


    VectorXd QPSolution_;
    bool first_run_ = true;
    bool first_lap_ = true;
    vector<geometry_msgs::Point> border_lines_;


    void getParameters(ros::NodeHandle& nh);
    void init_occupancy_grid();
    void init_SS_from_data(const string data_file);
    void map_callback(const nav_msgs::OccupancyGrid::Ptr &map_msg);
    void rrt_path_callback(const visualization_msgs::Marker::ConstPtr &path_msg);
    void visualize_centerline();
    int reset_QPSolution(int iter);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void state_callback(const sensor_msgs::Imu::ConstPtr &state_msg);
    void add_point();

    void solve_MPC(const Matrix<double,nx,1>& terminal_candidate);

    void get_linearized_dynamics(Matrix<double,nx,nx>& Ad, Matrix<double,nx, nu>& Bd, Matrix<double,nx,1>& hd,
            Matrix<double,nx,1>& x_op, Matrix<double,nu,1>& u_op, bool use_dyn);

    Vector3d global_to_track(double x, double y, double yaw, double s);
    Vector3d track_to_global(double e_y, double e_yaw, double s);

    void applyControl();
    void record_status();
    void visualize_mpc_solution(const vector<Sample>& convex_safe_set, const Matrix<double,nx,1>& terminal_candidate);

    Matrix<double,nx,1> select_terminal_candidate();
    void select_convex_safe_set(vector<Sample>& convex_safe_set, int iter_start, int iter_end, double s);
    int find_nearest_point(vector<Sample>& trajectory, double s);
    void update_cost_to_go(vector<Sample>& trajectory);
};

LMPC::LMPC(ros::NodeHandle &nh): nh_(nh){  // 构造函数

    getParameters(nh_);  // 从launch文件指引的lmpc_params.yaml提取参数
    run_num_ = 0;
    init_occupancy_grid();
    ROS_INFO("Begin ...");
    track_ = new Track(waypoint_file, true);   // 根据waypoint_file里的xy数据，建立等space的x,y,s,phi,kr
    //track_ = new Track(waypoint_file, map_, true);   // 根据waypoint_file里的xy数据，建立等space的x,y,s以及在地图里找左右边界范围
    ROS_INFO("End ...");
    odom_sub_ = nh_.subscribe(pose_topic, 10, &LMPC::odom_callback, this);  // 接收位置的topic信息
    state_sub_ = nh_.subscribe(state_topic, 10, &LMPC::state_callback, this);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);  // 发送drive的topic
    rrt_sub_ = nh_.subscribe("path_found", 1, &LMPC::rrt_path_callback, this);  
    map_sub_ = nh_.subscribe("map_updated", 1, &LMPC::map_callback, this);
    ROS_INFO("Begin 2 ...");
    track_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("track_centerline", 1);

    LMPC_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("LMPC", 1);
    pred_pub_ = nh.advertise<std_msgs::String>("Pred", 1);
    debugger_pub_ = nh_.advertise<visualization_msgs::Marker>("Debugger", 1);
    
    nav_msgs::Odometry odom_msg;
    sensor_msgs::Imu state_msg;
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;
    boost::shared_ptr<sensor_msgs::Imu const> state_ptr;
    ROS_INFO("Begin odom...");
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", ros::Duration(5));
    state_ptr = ros::topic::waitForMessage<sensor_msgs::Imu>("state_odom", ros::Duration(5));
    if (odom_ptr == nullptr){cout<< "fail to receive odom message!"<<endl;}
    else{
        odom_msg = *odom_ptr;
    }
    if (state_ptr == nullptr){cout << "fail to receive state_odom message!" <<endl;}
    else{
        state_msg = *state_ptr;
    }
    ROS_INFO("End odom");
    float x = odom_msg.pose.pose.position.x;
    float y = odom_msg.pose.pose.position.y;
    s_prev_ = track_->findTheta(x, y,0,true);  // 初始化s
    car_pos_ = tf::Vector3(x, y, 0.0);  // 初始化car pos
    yaw_ = tf::getYaw(odom_msg.pose.pose.orientation);   // 初始化航向角
    vel_ = sqrt(pow(odom_msg.twist.twist.linear.x,2) + pow(odom_msg.twist.twist.linear.y,2));  // 初始化车速
    yawdot_ = odom_msg.twist.twist.angular.z;
    slip_angle_ = atan2(odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.x);
    cout << "slip_angle_ = " << slip_angle_ << endl;
    Tf_ = state_msg.angular_velocity.x;
    TF_LAST = state_msg.angular_velocity.x;
    Tr_ = state_msg.angular_velocity.y;
    TR_LAST = state_msg.angular_velocity.y;
    delta_ = state_msg.angular_velocity.z;
    STEER_LAST = state_msg.angular_velocity.z;
    ax_ = state_msg.linear_acceleration.x;
    ay_ = state_msg.linear_acceleration.y;

    iter_ = 2;
    use_dyn_ = true;
    status_ = true;
    ROS_INFO("START_WORK");
    init_SS_from_data(init_data_file);
    lap_time_file_.open("/home/sun234/racing_work/src/LearningMPC-master/data/lap_time_set_10.csv");
    lap_ = 0;
    error_num_ = 0;

    input_dim_ = 6;
    ROS_INFO("here1");
    gp_learn_v_ = new GP_Learn(input_dim_, covfun1_, covfun2_, max_num_);
    gp_learn_r_ = new GP_Learn(input_dim_, covfun1_, covfun2_, max_num_);
    gp_learn_beta_ = new GP_Learn(input_dim_, covfun1_, covfun2_, max_num_);
    ROS_INFO("here2");
    ROS_INFO("ell_ = %f, sf_ = %f, sn_ = %f", ell_, sf_, sn_);
    params_ << ell_, sf_, sn_;
    ROS_INFO("here3");
    gp_learn_v_ ->SetParams(params_);
    gp_learn_r_ ->SetParams(params_);
    gp_learn_beta_ ->SetParams(params_);
    ROS_INFO("here4");
}

void LMPC::getParameters(ros::NodeHandle &nh) {
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("state_topic", state_topic);
    nh.getParam("drive_topic", drive_topic);
    nh.getParam("waypoint_file", waypoint_file);
    nh.getParam("init_data_file", init_data_file);

    nh.getParam("N",N);
    nh.getParam("Ts",Ts);
    nh.getParam("K_NEAR", K_NEAR);
    nh.getParam("ACCELERATION_MAX", ACCELERATION_MAX);
    nh.getParam("DECELERATION_MAX", DECELERATION_MAX);
    nh.getParam("SPEED_MAX", SPEED_MAX);
    nh.getParam("STEER_MAX", STEER_MAX);
    nh.getParam("DERT_TF_MAX", DERT_TF_MAX);
    nh.getParam("DERT_TR_MAX", DERT_TR_MAX);
    nh.getParam("DERT_STEER_MAX", DERT_STEER_MAX);
    nh.getParam("TF_MIN", TF_MIN);
    nh.getParam("TF_MAX", TF_MAX);
    nh.getParam("TR_MIN", TR_MIN);
    nh.getParam("TR_MAX", TR_MAX);
    nh.getParam("VEL_THRESHOLD", VEL_THRESHOLD);

    nh.getParam("WAYPOINT_SPACE", WAYPOINT_SPACE);
    nh.getParam("r_Tf",r_Tf);
    nh.getParam("r_Tr",r_Tr);
    nh.getParam("r_steer",r_steer);
    nh.getParam("q_s",q_s);
    nh.getParam("q_s_terminal", q_s_terminal);
    nh.getParam("q_time", q_time);
    nh.getParam("q_v", q_v);
    R.setZero();
    R.diagonal() << r_Tf, r_Tr, r_steer;
    nh.getParam("MAP_MARGIN",MAP_MARGIN);

    nh.getParam("wheelbase", car.wheelbase);
    nh.getParam("friction_coeff", car.friction_coeff);
    nh.getParam("height_cg", car.h_cg);
    nh.getParam("l_cg2rear", car.l_r);
    nh.getParam("l_cg2front", car.l_f);
    nh.getParam("Cf", car.Cf);
    nh.getParam("Cr", car.Cr);
    nh.getParam("Bf", car.Bf);
    nh.getParam("Br", car.Br);
    nh.getParam("Ef", car.Ef);
    nh.getParam("Er", car.Er);
    nh.getParam("R_wheel", car.R_wheel);
    nh.getParam("moment_inertia", car.I_z);
    nh.getParam("mass", car.mass);

    nh.getParam("cov_fun1", covfun1_);
    nh.getParam("cov_fun2", covfun2_);
    nh.getParam("ell", ell_);
    nh.getParam("sf", sf_);
    nh.getParam("sn", sn_);
    nh.getParam("max_num", max_num_);
}

int compare_s(Sample& s1, Sample& s2){
    return (s1.x(0)< s2.x(0));
}

void LMPC::init_occupancy_grid(){   // 初始化栅格地图
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));   // 栅格地图数据
    if (map_ptr == nullptr){ROS_INFO("No map received");}
    else{
        map_ = *map_ptr;
        map_updated_ = map_;
        ROS_INFO("Map received");
    }
    ROS_INFO("Initializing occupancy grid for map ...");
    occupancy_grid::inflate_map(map_, MAP_MARGIN);
    ROS_INFO("Finish Initializing occupancy grid for map");
}

void LMPC::init_SS_from_data(string data_file) {   // 从已有采样集文件构造SS_
    CSVReader reader(data_file);
    // Get the data from CSV File
    std::vector<std::vector<std::string>> dataList = reader.getData();
    SS_.clear();
    // Print the content of row by row on screen
    int time_prev=0;
    int it =0;
    vector<Sample> traj;
    for(std::vector<std::string> vec : dataList){
        Sample sample;
        sample.time = std::stof(vec.at(0));
        // check if it's a new lap
        if (sample.time - time_prev < 0) {
            it++;
            update_cost_to_go(traj);
            SS_.push_back(traj);
            traj.clear();
        }
        for (int i = 0; i < vec.size(); i ++){
            
        }
        cout << endl;
        sample.x(0) = std::stof(vec.at(1));   // x
        sample.x(1) = std::stof(vec.at(2));  // y
        sample.s = std::stof(vec.at(3));  // s
        sample.x(2) = std::stof(vec.at(4));  // yaw
        sample.x(3) = std::stof(vec.at(5));  // v
        sample.x(4) = std::stof(vec.at(6));  // r
        sample.x(5) = std::stof(vec.at(7));  // beta
        sample.u(0) = std::stof(vec.at(13));  // Tf_cmd
        sample.u(1) = std::stof(vec.at(14));  // Tr_cmd
        sample.u(2) = std::stof(vec.at(15));  // Steer_cmd
        sample.iter = it;
        traj.push_back(sample);
        time_prev = sample.time;
        ROS_INFO("time = %d", time_prev);
    }
    update_cost_to_go(traj);
    SS_.push_back(traj);
    ROS_INFO("END_work");
}

void LMPC::map_callback(const nav_msgs::OccupancyGrid::Ptr &map_msg){
    visualization_msgs::Marker dots;
    dots.header.frame_id = "map";
    dots.id = rviz_id::DEBUG;
    dots.ns = "debug_points";
    dots.type = visualization_msgs::Marker::POINTS;
    dots.scale.x = dots.scale.y = 0.1;
    dots.scale.z = 0.1;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.color.b = 1.0;
    dots.color.g = 0.5;
    dots.color.a = 1.0;

    vector<geometry_msgs::Point> path_processed;

    if(rrt_path_.empty()) return;
    for (int i=0; i< rrt_path_.size()-1; i++){
        path_processed.push_back(rrt_path_[i]);
        double dist = sqrt(pow(rrt_path_[i+1].x-rrt_path_[i].x, 2)
                           +pow(rrt_path_[i+1].y-rrt_path_[i].y, 2));
        if (dist < RRT_INTERVAL) continue;
        int num = static_cast<int>(ceil(dist/RRT_INTERVAL));
        for(int j=1; j< num; j++){
            geometry_msgs::Point p;
            p.x = rrt_path_[i].x + j*((rrt_path_[i+1].x - rrt_path_[i].x)/num);
            p.y = rrt_path_[i].y + j*((rrt_path_[i+1].y - rrt_path_[i].y)/num);
            path_processed.push_back(p);
        }
    }

    for (int i=0; i<path_processed.size(); i++){
        double theta = track_->findTheta(path_processed[i].x, path_processed[i].y, 0, true);
        Vector2d p_path(path_processed[i].x,path_processed[i].y);
        Vector2d p_proj(track_->x_eval(theta), track_->y_eval(theta));
        Vector2d p1, p2;
        int t=0;
        // search one direction until hit obstacle
        while(true){
            double x = (p_path + t*map_msg->info.resolution*(p_path-p_proj).normalized())(0);
            double y = (p_path + t*map_msg->info.resolution*(p_path-p_proj).normalized())(1);
            if(occupancy_grid::is_xy_occupied(*map_msg, x, y)){
                p1(0) = x; p1(1) = y;
                geometry_msgs::Point point;
                point.x = x; point.y = y;
                dots.points.push_back(point);
                break;
            }
            t++;
        }
        t=0;
        //search the other direction until hit obstacle
        while(true){
            double x = (p_path - t*map_msg->info.resolution*(p_path-p_proj).normalized())(0);
            double y = (p_path - t*map_msg->info.resolution*(p_path-p_proj).normalized())(1);
            if(occupancy_grid::is_xy_occupied(*map_msg, x, y)){
                p2(0) = x; p2(1) = y;
                geometry_msgs::Point point;
                point.x = x; point.y = y;
                dots.points.push_back(point);
                break;
            }
            t++;
        }
        double dx_dtheta = track_->x_eval_d(theta);
        double dy_dtheta = track_->y_eval_d(theta);
        double right_width = Vector2d(dy_dtheta, -dx_dtheta).dot(p1-p_proj)>0 ? (p1-p_proj).norm() : -(p1-p_proj).norm();
        double left_width = Vector2d(-dy_dtheta, dx_dtheta).dot(p2-p_proj)>0 ? (p2-p_proj).norm() : -(p2-p_proj).norm();
//        right_width = -0.15;
//        left_width =  0.4;
        cout<<"p1: "<< p1<<endl;
        cout<<"p2: "<< p2<<endl;

        track_->setHalfWidth(theta, left_width, right_width);
    }
    debugger_pub_.publish(dots);
}

void LMPC:: rrt_path_callback(const visualization_msgs::Marker::ConstPtr &path_msg){
    rrt_path_ = path_msg->points;
}

void LMPC::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){  // 获取当前状态信息
    if (marker_odom == 0){
        visualize_centerline();
    }
    /** process pose info **/
    float x = odom_msg->pose.pose.position.x;
    float y = odom_msg->pose.pose.position.y;  // 更新横纵坐标
    // 判断返回的信息是否有错误
    if (abs(x_ - x) > 10.0 || abs(y_ - y) > 10.0 || odom_msg->twist.twist.linear.x < 0.0){
        x = QPSolution_(nx);
        y = QPSolution_(nx + 1);
        s_curr_ = track_->findTheta(x, y,0,s_prev_,true);  // 当前状态的s
        car_pos_ = tf::Vector3(x, y, 0.0);
        x_ = x;
        y_ = y;
        yaw_ = QPSolution_(nx + 2);
        vel_ = QPSolution_(nx + 3);
        vx_ = QPSolution_(nx + 3) * cos(QPSolution_(nx + 5));
        vy_ =  QPSolution_(nx + 3) * sin(QPSolution_(nx + 5));
        yawdot_ = QPSolution_(nx + 4);
        slip_angle_ = QPSolution_(nx + 5);
    }
    else{
        s_curr_ = track_->findTheta(x, y,0,s_prev_,true);  // 当前状态的s
        car_pos_ = tf::Vector3(x, y, 0.0);
        x_ = x;
        y_ = y;
        yaw_ = tf::getYaw(odom_msg->pose.pose.orientation);
        vel_ = sqrt(pow(odom_msg->twist.twist.linear.x,2) + pow(odom_msg->twist.twist.linear.y,2));
        vx_ = odom_msg->twist.twist.linear.x;
        vy_ = odom_msg->twist.twist.linear.y;
        yawdot_ = odom_msg->twist.twist.angular.z;
        slip_angle_ = atan2(odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.x);
        cout << "slip_angle_ = " << slip_angle_ << endl;
    }
    if (first_run_){
        vx_error_ = 0.0;
    }else{
        vx_error_ = vx_ - (QPSolution_(nx + 3) * cos(QPSolution_(nx + 5)));
    }
    if (abs(vel_) < 1e-4){
        vel_ += 1e-4;
    }

    /** STATE MACHINE: check if dynamic model should be used based on current speed **/
    if ((!use_dyn_) && (vel_ > VEL_THRESHOLD)){
        use_dyn_ = true;
    }
    // if(use_dyn_ && (vel_< VEL_THRESHOLD*0.5)){
    //     use_dyn_ = false;
    // }
    if (vel_ > 20.0) {
        R(0,0) = 1.3 * r_Tf;
        R(1,1) = 1.3 * r_Tr;
        R(2,2) = 1.8 * r_steer;
    }
    marker_odom = 1;
}

void LMPC::state_callback(const sensor_msgs::Imu::ConstPtr &state_msg){
    Tf_ = state_msg->angular_velocity.x;
    Tr_ = state_msg->angular_velocity.y;
    delta_ = state_msg->angular_velocity.z;
    ax_ = state_msg->linear_acceleration.x;
    ay_ = state_msg->linear_acceleration.y;
    marker_state = 1;
    ROS_INFO("Tf_ = %lf, Tr_ = %lf, delta = %lf, ax_ = %lf, ay_ = %lf",Tf_, Tr_, delta_, ax_, ay_);
}

void LMPC::run(){
    if (first_run_){
        // initialize QPSolution_ from initial Sample Safe Set (using the 2nd iteration)
        reset_QPSolution(1);
    }

    /******** LMPC MAIN LOOP starts ********/

    /***check if it is new lap***/
    if (s_curr_ - s_prev_ < -track_->length/2 && !first_lap_){
        iter_++;
        update_cost_to_go(curr_trajectory_);   // 更新cost
        //sort(curr_trajectory_.begin(), curr_trajectory_.end(), compare_s);
        SS_.push_back(curr_trajectory_);  // 将一圈的存入SS_
        curr_trajectory_.clear();
     //   reset_QPSolution(iter_-1);
        time_ = 0;
    }

    if (s_curr_ - s_prev_ < -track_->length/2 && first_lap_){
        first_lap_ = false;
        curr_trajectory_.clear();
        time_ = 0;
    }
    if (!start_record_ && run_num_ <= 10){
        run_num_++;
    }else if(!start_record_ && run_num_ > 10){
        start_record_ = true;
    }

    if(!first_run_ && start_record_){
        x_input_[0] = last_vel_;
        x_input_[1] = last_yawdot_;
        x_input_[2] = last_slip_angle_;
        x_input_[3] = TF_LAST;
        x_input_[4] = TR_LAST;
        x_input_[5] = STEER_LAST;
        error_v_input_ = vel_ - (QPSolution_(nx + 3) - correct_v_0);
        error_r_input_ = yawdot_ - (QPSolution_(nx + 4) - correct_r_0);
        error_beta_input_ = slip_angle_ - (QPSolution_(nx + 5) - correct_beta_0);
        ROS_INFO("error_v_input_ = %f, error_r_input_ = %f, error_beta_input_  = %f", error_v_input_, error_r_input_,error_beta_input_);
        gp_learn_v_ ->AddPattern(x_input_, error_v_input_);
        gp_learn_r_ ->AddPattern(x_input_, error_r_input_);
        gp_learn_beta_ ->AddPattern(x_input_, error_beta_input_);
        ROS_INFO("AddPattern_finish");
    }

    /*** select terminal state candidate and its convex safe set ***/
    Matrix<double,nx,1> terminal_candidate = select_terminal_candidate();  // 选择z
    ROS_INFO("Terminal Candidate x = %lf, y = %lf, yaw = %lf, vel = %lf, r = %lf, beta = %lf", terminal_candidate(0), terminal_candidate(1),terminal_candidate(2), terminal_candidate(3), terminal_candidate(4), terminal_candidate(5));
    /** solve MPC and record current state***/
    status_ = true;
    for (int i=0; i<1; i++){
        solve_MPC(terminal_candidate);
    }
    if (status_){
        applyControl();   // 发送控制指令topic
    }else{
        error_num_++;
    }
    record_status();
    add_point();  // 将该时刻的状态量存入curr_trajectory_
    /*** store info and advance to next time step***/
    // for (int index_state = 0; index_state < N; ++index_state){
    //     std::cout << index_state << " state_v = " << QPSolution_.segment<nx>(index_state * nx)[3] << std::endl;
    // }
    terminal_state_pred_ = QPSolution_.segment<nx>(N*nx);  // 终端状态用于下一次迭代
    s_prev_ = s_curr_;
    time_++;
    first_run_ = false;
}

void LMPC::visualize_centerline(){
    visualization_msgs::Marker spline_dots;
    spline_dots.header.stamp = ros::Time::now();
    spline_dots.header.frame_id = "map";
    spline_dots.id = rviz_id::CENTERLINE_SPLINE;
    spline_dots.ns = "centerline";
    spline_dots.type = visualization_msgs::Marker::LINE_STRIP;
    spline_dots.scale.x = spline_dots.scale.y = 0.1;
    spline_dots.scale.z = 0.1;
    spline_dots.action = visualization_msgs::Marker::ADD;
    spline_dots.pose.orientation.w = 1.0;
    spline_dots.color.b = 1.0;
    spline_dots.color.a = 1.0;
    // spline_dots.lifetime = ros::Duration();

    for (float t=0.0; t<track_->length; t+=0.05){
        geometry_msgs::Point p;
        p.x = track_->x_eval(t);
        p.y = track_->y_eval(t);
        spline_dots.points.push_back(p);
    }

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(spline_dots);
    track_viz_pub_.publish(markers);
}

int LMPC::reset_QPSolution(int iter){
    QPSolution_ = VectorXd::Zero((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) + (terminal_nx));   // QPSolution (N+1)*nx[状态] + N * nu[控制] + nx * (N+1)[松弛] + 2 * (K_Near + 1)[权重]
    for (int i=0; i<N+1; i++){
        QPSolution_.segment<nx>(i*nx) = SS_[iter][i].x;
        if (i<N) QPSolution_.segment<nu>((N+1)*nx + i*nu) = SS_[iter][i].u;
    }
}

Matrix<double,nx,1> LMPC::select_terminal_candidate(){
    if (first_run_){   // 如果是t=0时，z_tj选上一圈的N时刻状态
        return SS_.back()[N].x;
    }
    else{
        // std::cout << "z.x = " << terminal_state_pred_[0] <<  "z.y = " << terminal_state_pred_[1] << "v_terminal = " << terminal_state_pred_[3] << std::endl;
        return terminal_state_pred_;
    }
}

void LMPC::add_point(){
    Sample point;
    point.x << car_pos_.x(), car_pos_.y(), yaw_, vel_,  yawdot_, slip_angle_;
    point.s = s_curr_;

    point.iter = iter_;
    point.time = time_;
    point.u = QPSolution_.segment<nu>((N+1)*nx);
    curr_trajectory_.push_back(point);
}

void LMPC::select_convex_safe_set(vector<Sample>& convex_safe_set, int iter_start, int iter_end, double s){
    std::cout << "iter_start = "<< iter_start << "iter_end = " << iter_end << std::endl; 
    for (int it = iter_start; it<= iter_end; it++){
        int nearest_ind = find_nearest_point(SS_[it], s);   // 找SS_集合第It圈中与s最近点的索引
        int start_ind, end_ind;
        int lap_cost = SS_[it][0].cost;  // 第it圈的总耗时/cost
        std::cout << "nearest_ind = "<< nearest_ind << std::endl; 

        if (K_NEAR%2 != 0 ) {   // 寻找nearest_ind的前后K_NEAR个点，K_near决定变化
            start_ind = nearest_ind - (K_NEAR-1)/2;
            end_ind = nearest_ind + (K_NEAR-1)/2;
        }
        else{
            start_ind = nearest_ind - K_NEAR/2 + 1;
            end_ind = nearest_ind + K_NEAR/2;
        }

        vector<Sample> curr_set;
        if (end_ind > SS_[it].size()-1){ // front portion of set crossed finishing line，如果采样安全集跨圈
            for (int ind=start_ind; ind<SS_[it].size(); ind++){
                curr_set.push_back(SS_[it][ind]);  
                // modify the cost-to-go for each point before finishing line
                // to incentivize the car to cross finishing line towards a new lap
                curr_set[curr_set.size()-1].cost += lap_cost;  // 这里的处理是不是应该加下一圈的cost,可能因为最新一圈的lap_cost没出来。
            }
            for (int ind=0; ind<end_ind-SS_[it].size()+1; ind ++){
                curr_set.push_back(SS_[it][ind]);
            }
            if (curr_set.size()!=K_NEAR) throw;  // for debug
        }
        else if (start_ind < 0){  //  set crossed finishing line， 起始点在上一圈，cost是到本圈末尾的cost
            for (int ind=start_ind+SS_[it].size(); ind<SS_[it].size(); ind++){
                // modify the cost-to-go, same
                curr_set.push_back(SS_[it][ind]);
                curr_set[curr_set.size()-1].cost += lap_cost;
            }
            for (int ind=0; ind<end_ind+1; ind ++){
                curr_set.push_back(SS_[it][ind]);
            }
            if (curr_set.size()!=K_NEAR) throw;  // for debug
        }
        else {  // no overlapping with finishing line
            for (int ind=start_ind; ind<=end_ind; ind++){
                curr_set.push_back(SS_[it][ind]);
                std::cout <<"ind = " << ind << "SS_[it][ind] x = " <<  SS_[it][ind].x[0] << " SS_[it][ind] y = "<< SS_[it][ind].x[1] << " SS_[it][ind] yaw = "<< SS_[it][ind].x[2] << " SS_[it][ind] v = " << SS_[it][ind].x[3] <<  " SS_[it][ind] r = " << SS_[it][ind].x[4] <<" SS_[it][ind] beta = " << SS_[it][ind].x[5] <<" SS_[it][ind] cost = " << SS_[it][ind].cost << std::endl;
            }
        }
        convex_safe_set.insert(convex_safe_set.end(), curr_set.begin(), curr_set.end());  // 将curr_set的数值insert到convex_safe_set里
    }
}

int LMPC::find_nearest_point(vector<Sample>& trajectory, double s){
    // binary search to find closest point to a given s，二分法
    int low = 0; int high = trajectory.size()-1;
    while (low<=high){
        int mid = (low + high)/2;
        if (abs(s - trajectory[mid].s) < 1e-3) return mid;
        if (s < trajectory[mid].s) high = mid-1;
        else low = mid+1;
    }
    return abs(trajectory[low].s-s) < (abs(trajectory[high].s-s))? low : high;   // 返回低与高中更接近s的索引
}

void LMPC::update_cost_to_go(vector<Sample>& trajectory){

    trajectory[trajectory.size()-1].cost = 0;  // 最后一个点的cost是0
    for (int i=trajectory.size()-2; i>=0; i--){   // 从倒数第二个点往前依次加1
        trajectory[i].cost = trajectory[i+1].cost + 1;
    }
    lap_time_file_ << trajectory[0].cost<<endl;
    lap_++;
    if (lap_ >= 20){
        lap_time_file_.close();
    }
}

Vector3d LMPC::global_to_track(double x, double y, double yaw, double s){
    double x_proj = track_->x_eval(s);
    double y_proj = track_->y_eval(s);
    double e_y = sqrt((x-x_proj)*(x-x_proj) + (y-y_proj)*(y-y_proj));
    double dx_ds = track_->x_eval_d(s);
    double dy_ds = track_->y_eval_d(s);
    e_y = dx_ds*(y-y_proj) - dy_ds*(x-x_proj) >0 ? e_y : -e_y;
    double e_yaw = yaw - atan2(dy_ds, dx_ds);
    while(e_yaw > M_PI) e_yaw -= 2*M_PI;
    while(e_yaw < -M_PI) e_yaw += 2*M_PI;

    return Vector3d(e_y, e_yaw, s);
}

Vector3d LMPC::track_to_global(double e_y, double e_yaw, double s){
    double dx_ds = track_->x_eval_d(s);
    double dy_ds = track_->y_eval_d(s);
    Vector2d proj(track_->x_eval(s), track_->y_eval(s));
    Vector2d pos = proj + Vector2d(-dy_ds, dx_ds).normalized()*e_y;
    double yaw = e_yaw + atan2(dy_ds, dx_ds);
    return Vector3d(pos(0), pos(1), yaw);
}

void LMPC::get_linearized_dynamics(Matrix<double,nx,nx>& Ad, Matrix<double,nx, nu>& Bd, Matrix<double,nx,1>& hd,
        Matrix<double,nx,1>& x_op, Matrix<double,nu,1>& u_op, bool use_dyn){
    
    double yaw = x_op(2);
    double v = x_op(3);
    if (abs(v - 0.0) < 1e-4){
        v += 0.0001;
    }
    double yaw_dot = x_op(4);
    double slip_angle = x_op(5);
    double T_f = u_op(0);
    double T_r = u_op(1);
    double delta = u_op(2);

    VectorXd dynamics(6), h(6);
    Matrix<double, nx, nx> A, M12;
    Matrix<double, nx, nu> B;

    // Single Track Dynamic Model

    double g = 9.81;
    double Fzf = car.mass * (g * car.l_r - ax_ * car.h_cg) / car.wheelbase;
    double Fzr = car.mass * (g * car.l_f + ax_ * car.h_cg) / car.wheelbase;

    dynamics(0) = v * cos(yaw+slip_angle);
    dynamics(1) = v * sin(yaw+slip_angle);
    dynamics(2) = yaw_dot;
    // cout << "Fzf = " << Fzf << endl;
    // cout << "Fzr = " << Fzr << endl;
    double Fxf = T_f / car.R_wheel;
    double Fxr = T_r / car.R_wheel;
    double alpha_f = -(atan((v * sin(slip_angle) + car.l_f * yaw_dot) / (v * cos(slip_angle))) - delta);
    double alpha_r =  -(atan((v * sin(slip_angle) - car.l_r * yaw_dot) / (v * cos(slip_angle))));
    cout << "v * sin(slip_angle) = " << v * sin(slip_angle) << endl;
    cout << "car.l_r * yaw_dot = " << car.l_r * yaw_dot << endl;
    cout << "alpha_f = " << alpha_f << endl;
    cout << "alpha_r = " << alpha_r << endl;
    cout << "delta = " << delta << endl;
    cout << "(v * sin(slip_angle) + car.l_f * yaw_dot) = " << v * sin(slip_angle) + car.l_f * yaw_dot << endl;
    cout << "atan((v * sin(slip_angle) + car.l_f * yaw_dot) / (v * cos(slip_angle))) = " << atan((v * sin(slip_angle) + car.l_f * yaw_dot) / (v * cos(slip_angle))) << endl;
    cout << "v * cos(slip_angle) = " << v * cos(slip_angle) << endl;
    // cout << "alpha_r = " << alpha_r << endl;
    double Fyf = car.friction_coeff * Fzf * sin(car.Cf * atan(car.Bf * alpha_f - car.Ef * (car.Bf * alpha_f - atan(car.Bf * alpha_f))));
    double Fyr = car.friction_coeff * Fzr * sin(car.Cr * atan(car.Br * alpha_r - car.Er * (car.Br * alpha_r - atan(car.Br * alpha_r))));
    // cout << "Fyf = " << Fyf << endl;
    // cout << "Fyr = " << Fyr << endl;
    cout << "Fxf * cos(delta) = " << Fxf * cos(delta) << endl;
    cout << "-Fyf * sin(delta) = " << -Fyf * sin(delta) << endl;
    cout << "Fxr = " << Fxr << endl;
    cout << "Fxf * sin(delta) = " << Fxf * sin(delta) << endl;
    cout << "Fyf * cos(delta) = " << Fyf * cos(delta) << endl;
    cout << "Fyr = " << Fyr << endl;
    dynamics(3) = (Fxf * cos(delta - slip_angle) - Fyf * sin(delta - slip_angle) + Fxr * cos(slip_angle) + Fyr * sin(slip_angle)) / car.mass;
    dynamics(4) = (car.l_f * (Fxf * sin(delta) + Fyf * cos(delta)) - car.l_r * Fyr) / car.I_z;
    dynamics(5) = -yaw_dot + (Fxf * sin(delta - slip_angle) + Fyf * cos(delta -  slip_angle) - Fxr * sin(slip_angle) + Fyr * cos(slip_angle)) / (car.mass * v);

    cout << "dynamic = " << dynamics << endl;

    double dfx_dyaw, dfx_dv, dfx_dslip, dfy_dyaw, dfy_dv, dfy_dslip, dfv_dv, dfv_dyawdot, 
        dfv_dslip, dfv_dTf, dfv_dTr, dfv_ddelta, dfyawdot_dv, dfyawdot_dyawdot,dfyawdot_dslip,  dfyawdot_dTf, dfyawdot_ddelta, dfslip_dv, 
        dfslip_dyawdot, dfslip_dslip, dfslip_dTf, dfslip_dTr, dfslip_ddelta;

    dfx_dyaw = -v * sin(yaw + slip_angle);
    dfx_dv = cos(yaw + slip_angle);
    dfx_dslip = -v * sin(slip_angle + yaw);

    // ROS_INFO("dfx_dyaw = %lf", dfx_dyaw);
    // ROS_INFO("dfx_dv = %lf", dfx_dv);
    // ROS_INFO("dfx_dslip = %lf", dfx_dslip);
   
    dfy_dyaw = v * cos(slip_angle + yaw);
    dfy_dv = sin(slip_angle + yaw);
    dfy_dslip = v * cos(slip_angle + yaw);

    // ROS_INFO("dfy_dyaw = %lf", dfy_dyaw);
    // ROS_INFO("dfy_dv = %lf", dfy_dv);
    // ROS_INFO("dfy_dslip = %lf", dfy_dslip);

double t2 = cos(slip_angle);
double t3 = cos(delta);
double t4 = sin(slip_angle);
double t5 = car.l_f*yaw_dot;
double t6 = car.l_r*yaw_dot;
double t7 = slip_angle+yaw;
double t8 = car.Bf * car.Bf;
double t9 = car.Br * car.Br;
double t12 = 1.0/car.I_z;
double t14 = 1.0/car.R_wheel;
double t15 = -delta;
double t16 = 1.0/car.mass;
double t17 = 1.0/v;
double t10 = cos(t7);
double t11 = sin(t7);
double t13 = t4*v;
double t18 = t17 * t17;
double t19 = 1.0/t2;
double t23 = slip_angle+t15;
double t28 = T_r*t4*t14;
double t20 = t19 * t19;
double t21 = t10*v;
double t22 = t11*v;
double t24 = -t13;
double t25 = cos(t23);
double t26 = sin(t23);
double t27 = t5+t13;
double t32 = t4*t17*t19;
double t29 = -t22;
double t30 = t6+t24;
double t31 = t27 * t27;
double t34 = T_f*t14*t26;
double t35 = t17*t19*t27;
double t36 = t18*t19*t27;
double t38 = t4*t17*t20*t27;
double t33 = t30 * t30;
double t37 = atan(t35);
double t39 = -t36;
double t40 = t17*t19*t30;
double t41 = t18*t19*t30;
double t42 = t18*t20*t31;
double t46 = t4*t17*t20*t30;
double t47 = t38+1.0;
double t43 = atan(t40);
double t44 = t42+1.0;
double t45 = -t37;
double t48 = t18*t20*t33;
double t56 = t46-1.0;
double t67 = t32+t39;
double t68 = t32+t41;
double t49 = t43 * t43;
double t50 = car.Br*t43;
double t51 = delta+t45;
double t52 = t48+1.0;
double t55 = 1.0/t44;
double t53 = atan(t50);
double t54 = -t50;
double t57 = t51 * t51;
double t58 = car.Bf*t51;
double t60 = t9*t49;
double t61 = 1.0/t52;
double t70 = car.Bf*car.l_f*t17*t19*t55;
double t74 = car.Bf*t47*t55;
double t82 = car.Bf*t55*t67;
double t59 = atan(t58);
double t62 = -t58;
double t63 = t60+1.0;
double t64 = t8*t57;
double t71 = -t70;
double t72 = car.Br*car.l_r*t17*t19*t61;
double t75 = t53+t54;
double t76 = -t74;
double t77 = -car.Er*(t50-t53);
double t78 = car.Br*t56*t61;
double t83 = -t82;
double t84 = car.Br*t61*t68;
double t65 = 1.0/t63;
double t66 = t64+1.0;
double t73 = -t72;
double t79 = t59+t62;
double t80 = -t78;
double t81 = -car.Ef*(t58-t59);
double t85 = -t84;
double t90 = t50+t77;
double t69 = 1.0/t66;
double t86 = t65*t72;
double t88 = t65*t73;
double t91 = atan(t90);
double t92 = t90 * t90;
double t100 = t58+t81;
double t103 = t65*t78;
double t107 = t65*t80;
double t112 = t65*t84;
double t114 = t65*t85;
double t87 = t69*t70;
double t89 = t69*t71;
double t93 = car.Cr*t91;
double t94 = t92+1.0;
double t98 = t69*t74;
double t101 = t69*t76;
double t102 = atan(t100);
double t104 = t100 * t100;
double t113 = t69*t82;
double t115 = t69*t83;
double t119 = t72+t88;
double t124 = t78+t107;
double t129 = t84+t114;
double t95 = cos(t93);
double t96 = sin(t93);
double t97 = 1.0/t94;
double t106 = car.Cf*t102;
double t110 = t104+1.0;
double t118 = t70+t89;
double t121 = car.Er*t119;
double t122 = t74+t101;
double t125 = car.Er*t124;
double t128 = t82+t115;
double t131 = car.Er*t129;
double t99 = Fzr*car.friction_coeff*t2*t96;
double t108 = cos(t106);
double t109 = sin(t106);
double t111 = 1.0/t110;
double t120 = car.Ef*t118;
double t123 = car.Ef*t122;
double t127 = t73+t121;
double t130 = car.Ef*t128;
double t133 = t80+t125;
double t135 = t85+t131;
double t105 = -t99;
double t116 = Fzf*car.friction_coeff*t25*t109;
double t126 = t71+t120;
double t132 = t76+t123;
double t134 = t83+t130;
double t117 = -t116;

double tu2 = cos(slip_angle);
double tu3 = cos(delta);
double tu4 = sin(slip_angle);
double tu5 = sin(delta);
double tu6 = car.l_f*yaw_dot;
double tu7 = car.Bf * car.Bf;
double tu8 = 1.0/car.I_z;
double tu10 = 1.0/car.R_wheel;
double tu11 = -delta;
double tu12 = 1.0/car.mass;
double tu13 = 1.0/v;
double tu9 =  tu4*v;
double tu14 = 1.0/ tu2;
double tu15 = slip_angle+ tu11;
double tu16 = cos( tu15);
double tu17 = sin( tu15);
double tu18 =  tu6+ tu9;
double tu19 = tu13*tu14*tu18;
double tu20 = atan(tu19);
double tu21 = -tu20;
double tu22 = delta+tu21;
double tu23 = tu22 * tu22;
double tu24 = car.Bf*tu22;
double tu25 = atan(tu24);
double tu26 = - tu24;
double tu27 =  tu7* tu23;
double tu28 =  tu27+1.0;
double tu36 =  tu25+ tu26;
double tu37 = -car.Ef*( tu24- tu25);
double tu29 = 1.0/ tu28;
double tu38 =  tu24+ tu37;
double tu30 = car.Bf* tu29;
double tu39 = atan( tu38);
double tu40 =  tu38 * tu38;
double tu31 = - tu30;
double tu41 = car.Cf* tu39;
double tu44 =  tu40+1.0;
double tu32 = car.Bf+ tu31;
double tu42 = cos( tu41);
double tu43 = sin( tu41);
double tu45 = 1.0/ tu44;
double tu33 = car.Ef* tu32;
double tu34 = - tu33;
double tu35 = car.Bf+ tu34;


    dfv_dv = -t16*(car.Cf*Fzf*car.friction_coeff*t26*t108*t111*(t82-t130)+car.Cr*Fzr*car.friction_coeff*t4*t95*t97*(t84-t131));
    dfv_dyawdot = -t16*(car.Cf*Fzf*car.friction_coeff*t26*t108*t111*(t70-t120)-car.Cr*Fzr*car.friction_coeff*t4*t95*t97*(t72-t121));
    dfv_dslip =-t16*(t28+t34+t105+t117+car.Cf*Fzf*car.friction_coeff*t26*t108*t111*(t74-t123)-car.Cr*Fzr*car.friction_coeff*t4*t95*t97*(t78-t125));
    dfv_dTf = tu10*tu12*tu16;
    dfv_dTr = tu2*tu10*tu12;
    dfv_ddelta = tu12*(T_f*tu10*tu17-Fzf*car.friction_coeff*tu16*tu43+car.Cf*Fzf*car.friction_coeff*tu17*tu35*tu42*tu45);
    // ROS_INFO("dfv_dv = %lf", dfv_dv);
    // ROS_INFO("dfv_dyawdot = %lf", dfv_dyawdot);
    // ROS_INFO("dfv_dslip = %lf", dfv_dslip);
    // ROS_INFO("dfv_dTf = %lf", dfv_dTf);
    // ROS_INFO("dfv_dTr = %lf", dfv_dTr);
    // ROS_INFO("dfv_ddelta = %lf", dfv_ddelta);
    // ROS_INFO("dfv_dax = %lf", dfv_dax);

    dfyawdot_dv = t12*(car.Cr*Fzr*car.l_r*car.friction_coeff*t95*t97*(t84-t131)-car.Cf*Fzf*car.l_f*car.friction_coeff*t3*t108*t111*(t82-t130));
    dfyawdot_dyawdot = -t12*(car.Cr*Fzr*car.l_r*car.friction_coeff*t95*t97*(t72-t121)+car.Cf*Fzf*car.l_f*car.friction_coeff*t3*t108*t111*(t70-t120));
    dfyawdot_dslip = -t12*(car.Cr*Fzr*car.l_r*car.friction_coeff*t95*t97*(t78-t125)+car.Cf*Fzf*car.l_f*car.friction_coeff*t3*t108*t111*(t74-t123));
    dfyawdot_dTf = car.l_f*tu5*tu8*tu10;
    dfyawdot_ddelta = car.l_f*tu8*(T_f*tu3*tu10-Fzf*car.friction_coeff*tu5*tu43+car.Cf*Fzf*car.friction_coeff*tu3*tu35*tu42*tu45);

    // ROS_INFO("dfyawdot_dv = %lf", dfyawdot_dv);
    // ROS_INFO("dfyawdot_dyawdot = %lf", dfyawdot_dyawdot);
    // ROS_INFO("dfyawdot_dslip = %lf", dfyawdot_dslip);
    // ROS_INFO("dfyawdot_dTf = %lf", dfyawdot_dTf);
    // ROS_INFO("dfyawdot_ddelta = %lf", dfyawdot_ddelta);
    // ROS_INFO("dfyawdot_dax = %lf", dfyawdot_dax);

    dfslip_dv = -t16*t17*(car.Cf*Fzf*car.friction_coeff*t25*t108*t111*(t82-t130)+car.Cr*Fzr*car.friction_coeff*t2*t95*t97*(t84-t131))+t16*t18*(t28+t34+t105+t117);
    dfslip_dyawdot = -t16*t17*(car.Cf*Fzf*car.friction_coeff*t25*t108*t111*(t70-t120)-car.Cr*Fzr*car.friction_coeff*t2*t95*t97*(t72-t121))-1.0;
    dfslip_dslip = -t16*t17*(T_f*t14*t25+T_r*t2*t14+Fzf*car.friction_coeff*t26*t109+Fzr*car.friction_coeff*t4*t96+car.Cf*Fzf*car.friction_coeff*t25*t108*t111*(t74-t123)-car.Cr*Fzr*car.friction_coeff*t2*t95*t97*(t78-t125));
    dfslip_dTf = -tu10*tu12*tu13*tu17;
    dfslip_dTr = -tu4*tu10*tu12*tu13;
    dfslip_ddelta = tu12*tu13*(T_f*tu10*tu16+Fzf*car.friction_coeff*tu17*tu43+car.Cf*Fzf*car.friction_coeff*tu16*tu35*tu42*tu45);

    // ROS_INFO("dfslip_dv = %lf", dfslip_dv);
    // ROS_INFO("dfslip_dyawdot = %lf", dfslip_dyawdot);
    // ROS_INFO("dfslip_dslip = %lf", dfslip_dslip);
    // ROS_INFO("dfslip_dTf = %lf", dfslip_dTf);
    // ROS_INFO("dfslip_dTr = %lf", dfslip_dTr);
    // ROS_INFO("dfslip_ddelta = %lf", dfslip_ddelta);
    // ROS_INFO("dfslip_dax = %lf", dfslip_dax);

    A <<    0.0, 0.0, dfx_dyaw, dfx_dv, 0.0,  dfx_dslip,
            0.0,           0.0,    dfy_dyaw, dfy_dv,0.0,  dfy_dslip,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0,           0.0,               0.0,           dfv_dv, dfv_dyawdot, dfv_dslip,
            0.0,           0.0,               0.0,           dfyawdot_dv,     dfyawdot_dyawdot,    dfyawdot_dslip,
            0.0,           0.0,               0.0,             dfslip_dv,       dfslip_dyawdot,             dfslip_dslip;

    B <<    0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            dfv_dTf, dfv_dTr, dfv_ddelta,
            dfyawdot_dTf, 0.0, dfyawdot_ddelta,
            dfslip_dTf,   dfslip_dTr,   dfslip_ddelta;

    /**  Discretize using Zero-Order Hold **/
    Matrix<double,nx+nx,nx+nx> aux, M;
    aux.setZero();
    aux.block<nx,nx>(0,0) << A;
    aux.block<nx,nx>(0, nx) << Matrix<double,nx,nx>::Identity();
    M = (aux*Ts).exp();
    M12 = M.block<nx,nx>(0,nx);
    h = dynamics - (A*x_op + B*u_op);

    Ad = (A*Ts).exp();
    Bd = M12*B;
    hd = M12*h;

    if (isi0_){
        add_state_pred_ << 0.0, 0.0, 0.0, correct_v_0, correct_r_0, correct_beta_0;
        isi0_ = false;
    }else{
        x_now_input_[0] = v;
        x_now_input_[1] = yaw_dot;
        x_now_input_[2] = slip_angle;
        x_now_input_[3] = T_f;
        x_now_input_[4] = T_r;
        x_now_input_[5] = delta;

        add_state_pred_ << 0.0, 0.0, 0.0, gp_learn_v_->CalculateMu(x_now_input_), gp_learn_r_->CalculateMu(x_now_input_), gp_learn_beta_->CalculateMu(x_now_input_);

    }
    ROS_INFO("v_add = %f", gp_learn_v_->CalculateMu(x_now_input_));
    ROS_INFO("r_add = %f", gp_learn_r_->CalculateMu(x_now_input_));
    ROS_INFO("beta_add = %f", gp_learn_beta_->CalculateMu(x_now_input_));

    hd = hd + add_state_pred_;
    // cout << "A = " << A << endl;
    // cout << "B = " << B << endl;
    // cout << "Ad = " << Ad << endl;
    // cout << "Bd = " << Bd << endl;
    // cout << "hd = " << hd << endl;
}

void wrap_angle(double& angle, const double angle_ref){
    while(angle - angle_ref > M_PI) {angle -= 2*M_PI;}
    while(angle - angle_ref < -M_PI) {angle += 2*M_PI;}
}

void LMPC::solve_MPC(const Matrix<double,nx,1>& terminal_candidate){
    vector<Sample> terminal_CSS;  // 终端凸区间
    double s_t = track_->findTheta(terminal_candidate(0), terminal_candidate(1), 0, true);   // 找z的s
    select_convex_safe_set(terminal_CSS, iter_-2, iter_-1, s_t);  // 在前两圈中根据z选择terminal_CSS
    ROS_INFO("HERE");
    /** MPC variables: z = [x0, ..., xN, u0, ..., uN-1, s0, ..., sN, lambda0, ....., lambda(2*K_NEAR), s_t1, s_t2, .. s_t6]*
     *  constraints: dynamics, track bounds, input limits, acceleration limit, slack, lambdas, terminal state, sum of lambda's*/
    SparseMatrix<double> HessianMatrix((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);
    SparseMatrix<double> constraintMatrix((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);

    VectorXd gradient((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx);

    VectorXd lower((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx);
    VectorXd upper((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx);

    gradient.setZero();
    lower.setZero(); upper.setZero();

    Matrix<double,nx,1> x_k_ref;
    Matrix<double,nu,1> u_k_ref;
    Matrix<double,nx,nx> Ad;
    Matrix<double,nx,nu> Bd;
    Matrix<double,nx,1> x0, hd;
    border_lines_.clear();

    x0 << car_pos_.x(), car_pos_.y(),  yaw_, vel_, yawdot_, slip_angle_;
    cout << "x0 = " << x0 << endl;
    // if (use_dyn_){x0 << s_curr_, s_n_,  yaw_, vel_, yawdot_, slip_angle_, Tf_, Tr_, delta_, ax_, ay_;}
    // else{ x0 << s_curr_, s_n_, yaw_, vel_, 0.0, 0.0,  Tf_, Tr_, delta_, ax_, ay_;}
    /** make sure there are no discontinuities in yaw**/
    // first check terminal safe_set
    for (int i=0; i<terminal_CSS.size(); i++){
        wrap_angle(terminal_CSS[i].x(2), x0(2));  // 检查是否在-Pi到Pi之间
    }
    // also check for previous QPSolution
    for (int i=0; i<N+1; i++){
        wrap_angle(QPSolution_(i*nx+2), x0(2));
    }
    double s_ref = 0.0;
    // cout << "QP_solution_ = " << QPSolution_ << endl;
    for (int i=0; i<N+1; i++){        //0 to N
        if (i < N){
            x_k_ref = QPSolution_.segment<nx>((i)*nx);
            u_k_ref = QPSolution_.segment<nu>((N+1)*nx + (i)*nu);
        }else{
            x_k_ref = QPSolution_.segment<nx>(i*nx);
            u_k_ref = QPSolution_.segment<nu>((N+1)*nx + (N - 1)*nu);
        }
        if (i == 0){
            s_ref = track_->findTheta(x_k_ref(0), x_k_ref(1), 0, true);
        }else{
            s_ref = track_->findTheta(x_k_ref(0), x_k_ref(1), 0, s_ref_cur_, true);  // 找s_ref，利用上一采样点的预测时域内状态作为泰勒展开参考点
        }
        ROS_INFO("S_ref = %lf, X_ref = %lf, Y_ref = %lf, Yaw_ref = %lf, Vel_ref = %lf, Yaw_rate_ref = %lf, Beta_ref = %lf", s_ref, x_k_ref(0),  x_k_ref(1),  x_k_ref(2),  x_k_ref(3),  x_k_ref(4),  x_k_ref(5));
        s_ref_cur_ = s_ref;
        if (i == 0){
            x_now_input_[0] = vel_;
            x_now_input_[1] = yawdot_;
            x_now_input_[2] = slip_angle_;
            x_now_input_[3] = TF_LAST;
            x_now_input_[4] = TR_LAST;
            x_now_input_[5] = STEER_LAST;
            correct_v_0 = gp_learn_v_->CalculateMu(x_now_input_);
            correct_r_0 = gp_learn_r_->CalculateMu(x_now_input_);
            correct_beta_0 = gp_learn_beta_->CalculateMu(x_now_input_);
            isi0_ = true;
        }
        get_linearized_dynamics(Ad, Bd, hd, x_k_ref, u_k_ref, use_dyn_);   // 计算0阶离散线性系统矩阵Ad, Bd, hd
        ROS_INFO("HEREdone");
        /* form Hessian entries*/
        // cost does not depend on x0, only 1 to N
        if (i > 0){
            HessianMatrix.insert((N+1)*nx + N*nu + i, (N+1)*nx + N*nu + i) = q_s;
        }
        if (i<N){
            for (int row=0; row<nu; row++){
                HessianMatrix.insert((N+1)*nx + i*nu + row, (N+1)*nx + i*nu + row) = R(row, row);
            }
        }

        /* form constraint matrix */
        if (i<N){
            // Ad
            for (int row=0; row<nx; row++){
                for(int col=0; col<nx; col++){
                    constraintMatrix.insert((i+1)*nx+row, i*nx+col) = Ad(row,col);
                }
            }
            // Bd
            for (int row=0; row<nx; row++){
                for(int col=0; col<nu; col++){
                    constraintMatrix.insert((i+1)*nx+row, (N+1)*nx+ i*nu+col) = Bd(row,col);
                }
            }
            lower.segment<nx>((i+1)*nx) = -hd;//-OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY;//-hd;
            upper.segment<nx>((i+1)*nx) = -hd; //OsqpEigen::INFTY, OsqpEigen::INFTY,OsqpEigen::INFTY,OsqpEigen::INFTY;//-hd;
        }

        // -I for each x_k+1
        for (int row=0; row<nx; row++) {
            constraintMatrix.insert(i*nx+row, i*nx+row) = -1.0;
        }

        double dx_dtheta = track_->x_eval_d(s_ref);
        double dy_dtheta = track_->y_eval_d(s_ref);

        constraintMatrix.insert((N+1)*nx+ 2*i, i*nx) = -dy_dtheta;      // a*x
        constraintMatrix.insert((N+1)*nx+ 2*i, i*nx+1) = dx_dtheta;     // b*y
        constraintMatrix.insert((N+1)*nx+ 2*i, (N+1)*nx +N*nu +i) = 1.0;   // min(C1,C2) <= a*x + b*y + s_k <= inf

        constraintMatrix.insert((N+1)*nx+ 2*i+1, i*nx) = -dy_dtheta;      // a*x
        constraintMatrix.insert((N+1)*nx+ 2*i+1, i*nx+1) = dx_dtheta;     // b*y
        constraintMatrix.insert((N+1)*nx+ 2*i+1, (N+1)*nx +N*nu +i) = -1.0;   // -inf <= a*x + b*y - s_k <= max(C1,C2)
        cout << "dy_dtheta = " << dy_dtheta << ", " << "dx_dtheta = " << dx_dtheta << endl;

        //get upper line and lower line
        Vector2d left_tangent_p, right_tangent_p, center_p;
        Vector2d right_line_p1, right_line_p2, left_line_p1, left_line_p2;
        geometry_msgs::Point r_p1, r_p2, l_p1, l_p2;

        center_p << track_->x_eval(s_ref), track_->y_eval(s_ref);
        right_tangent_p = center_p + track_->getRightHalfWidth(s_ref) * Vector2d(dy_dtheta, -dx_dtheta).normalized();   // centerp投影点
        left_tangent_p  = center_p + track_->getLeftHalfWidth(s_ref) * Vector2d(-dy_dtheta, dx_dtheta).normalized();

        right_line_p1 = right_tangent_p + 0.15*Vector2d(dx_dtheta, dy_dtheta).normalized();
        right_line_p2 = right_tangent_p - 0.15*Vector2d(dx_dtheta, dy_dtheta).normalized();
        left_line_p1 = left_tangent_p + 0.15*Vector2d(dx_dtheta, dy_dtheta).normalized();
        left_line_p2 = left_tangent_p - 0.15*Vector2d(dx_dtheta, dy_dtheta).normalized();

        // For visualizing track boundaries
        r_p1.x = right_line_p1(0);  r_p1.y = right_line_p1(1);
        r_p2.x = right_line_p2(0);  r_p2.y = right_line_p2(1);
        l_p1.x = left_line_p1(0);   l_p1.y = left_line_p1(1);
        l_p2.x = left_line_p2(0);   l_p2.y = left_line_p2(1);
        border_lines_.push_back(r_p1);  border_lines_.push_back(r_p2);
        border_lines_.push_back(l_p1); border_lines_.push_back(l_p2);

        double C1 =  - dy_dtheta*right_tangent_p(0) + dx_dtheta*right_tangent_p(1);
        double C2 = - dy_dtheta*left_tangent_p(0) + dx_dtheta*left_tangent_p(1);

        lower((N+1)*nx+ 2*i) =  min(C1, C2);
        std::cout << "min(C1, C2) = " << min(C1, C2) << std::endl;
        upper((N+1)*nx+ 2*i) = OsqpEigen::INFTY;

        lower((N+1)*nx+ 2*i+1) = -OsqpEigen::INFTY;
        upper((N+1)*nx+ 2*i+1) = max(C1, C2);
        std::cout << "max(C1, C2) = " << max(C1, C2) << std::endl;

        // u_min < u < u_max
        if (i<N){
            for (int row=0; row<nu; row++){
                constraintMatrix.insert((N+1)*nx+ 2*(N+1) +i*nu+row, (N+1)*nx+i*nu+row) = 1.0;
            }
            // input bounds: speed and steer
            lower.segment<nu>((N+1)*nx+ 2*(N+1) +i*nu) <<  - TF_MAX, -TR_MAX, -STEER_MAX;
            upper.segment<nu>((N+1)*nx+ 2*(N+1) +i*nu) << TF_MAX, TR_MAX, STEER_MAX;
        }

        //max velocity
        constraintMatrix.insert((N+1)*nx+ 2*(N+1) + N*nu +i, i*nx+3) = 1;
        lower((N+1)*nx+ 2*(N+1) + N*nu +i) = 0;
        upper((N+1)*nx+ 2*(N+1) + N*nu +i) = SPEED_MAX;

        //max yaw_dot
        constraintMatrix.insert((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i, i*nx+4) = 1;
        lower((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = -car.friction_coeff * 9.81 / abs(x_k_ref(3) * cos(x_k_ref(5)));
        // lower((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = -OsqpEigen::INFTY;
        ROS_INFO("MIN YAW DOT = %lf", -car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5))));
        upper((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = car.friction_coeff * 9.81 / abs(x_k_ref(3) * cos(x_k_ref(5)));
        // upper((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = OsqpEigen::INFTY;
        ROS_INFO("MAX YAW DOT = %lf", car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5))));

        // max slip angle
        constraintMatrix.insert((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i, i*nx+5) = 1;
        lower((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = -atan(0.02 * car.friction_coeff * 9.81);
        upper((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = atan(0.02 * car.friction_coeff * 9.81);
        // lower((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = -OsqpEigen::INFTY;
        // upper((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = OsqpEigen::INFTY;
        // ROS_INFO("MIN SLIP ANGLE = %lf", -atan(0.02 * car.friction_coeff * 9.81));
        // ROS_INFO("MAX SLIP ANGLE = %lf", atan(0.02 * car.friction_coeff * 9.81));

        // s_k >= 0
        constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu +3* (N+1) + i, (N+1)*nx+N*nu +i) = 1.0;
        lower((N+1)*nx + 2*(N+1) + N*nu  + 3 * (N+1) + i) = 0;
        upper((N+1)*nx + 2*(N+1) + N*nu  + 3 * (N+1) + i) = OsqpEigen::INFTY;

        // max dertTf
        if (i < N && i > 0){
            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i, (N + 1)*nx + i * nu) = 1.0;
            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i, (N + 1)*nx + (i - 1) * nu) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i) = -DERT_TF_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i) = DERT_TF_MAX * Ts;

            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i, (N + 1)*nx + i * nu + 1) = 1.0;
            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i, (N + 1)*nx + (i - 1) * nu + 1) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i) = -DERT_TR_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i) = DERT_TR_MAX * Ts;

            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i, (N + 1)*nx + i * nu + 2) = 1.0;
            constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i, (N + 1)*nx + (i - 1) * nu + 2) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i) = -DERT_STEER_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i) = DERT_STEER_MAX * Ts;

        } 
    }
    constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1), (N + 1)*nx) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1)) = TF_LAST - DERT_TF_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1)) = TF_LAST + DERT_TF_MAX * Ts;

    constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N, (N + 1)*nx + 1) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N) = TR_LAST - DERT_TR_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N) = TR_LAST + DERT_TR_MAX * Ts;

    constraintMatrix.insert((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N, (N + 1)*nx + 2) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N) = STEER_LAST - DERT_STEER_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N) = STEER_LAST + DERT_STEER_MAX * Ts;
    int numOfConstraintsSoFar = (N+1)*nx + 2*(N+1) + N*nu + 3* (N+1) + (N+1) + 3 * N;

    // lamda's >= 0
    for (int i=0; i<2*K_NEAR; i++){
        constraintMatrix.insert(numOfConstraintsSoFar + i, (N+1)*nx+ N*nu + (N+1) + i) = 1.0;
        lower(numOfConstraintsSoFar + i) = 0;
        upper(numOfConstraintsSoFar + i) = 1.0;
    }
    numOfConstraintsSoFar += 2*K_NEAR;

    // terminal state constraints:  -s_t <= -x_N+1 + linear_combination(lambda's) <= s_t
    // 0 <= s_t -x_N+1 + linear_combination(lambda's) <= inf
    for (int i=0; i<2*K_NEAR; i++){
        for (int state_ind=0; state_ind<terminal_nx; state_ind++){
            constraintMatrix.insert(numOfConstraintsSoFar + state_ind, (N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].x(state_ind);
        }
    }
    for (int state_ind=0; state_ind<terminal_nx; state_ind++){
        constraintMatrix.insert(numOfConstraintsSoFar + state_ind, N*nx + state_ind) = -1;
        constraintMatrix.insert(numOfConstraintsSoFar+state_ind, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + state_ind) = 1;
        lower(numOfConstraintsSoFar+state_ind) = 0.0;
        upper(numOfConstraintsSoFar+state_ind) = OsqpEigen::INFTY;
    }
    numOfConstraintsSoFar += terminal_nx;

    //-inf <= -x_N+1 + linear_combination(lambda's) - s_t <= 0
    for (int i=0; i<2*K_NEAR; i++){
        for (int state_ind=0; state_ind<terminal_nx; state_ind++){
            constraintMatrix.insert(numOfConstraintsSoFar + state_ind, (N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].x(state_ind);
        }
    }
    for (int state_ind=0; state_ind<terminal_nx; state_ind++){
        constraintMatrix.insert(numOfConstraintsSoFar + state_ind, N*nx + state_ind) = -1;
        constraintMatrix.insert(numOfConstraintsSoFar+state_ind, (N+1)*nx + N*nu + (N+1) + 2*K_NEAR + state_ind) = -1;
        lower(numOfConstraintsSoFar+state_ind) = -OsqpEigen::INFTY;
        upper(numOfConstraintsSoFar+state_ind) = 0;
    }

    numOfConstraintsSoFar += terminal_nx;
   // cout<<"con dim: "<< (N+1)*nx+ 2*(N+1)*nx + N*nu + (N-1) + (N+1)*nx + (2*K_NEAR) + nx+1 <<endl;
    // sum of lamda's = 1;
    for (int i=0; i<2*K_NEAR; i++){
        constraintMatrix.insert(numOfConstraintsSoFar, (N+1)*nx+ N*nu + (N+1) + i) = 1;
    }

    lower(numOfConstraintsSoFar) = 1.0;
    upper(numOfConstraintsSoFar) = 1.0;
    numOfConstraintsSoFar++;

    for(int i = 0; i < terminal_nx ; i++){
        constraintMatrix.insert(numOfConstraintsSoFar + i, (N+1)*nx + N*nu + (N+1) + 2*K_NEAR + i) = 1.0;
        lower(numOfConstraintsSoFar + i) = 0;
        upper(numOfConstraintsSoFar + i) = OsqpEigen::INFTY;
    }
    numOfConstraintsSoFar += terminal_nx;
    if (numOfConstraintsSoFar != (N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx) throw;  // for debug
    // gradient
    int lowest_cost1 = terminal_CSS[K_NEAR-1].cost;
    int lowest_cost2 = terminal_CSS[2*K_NEAR-1].cost;
    // for (int i=0; i<K_NEAR; i++){
    //     gradient((N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].cost-lowest_cost1;
    // }
    for (int i=0; i<K_NEAR; i++){
        gradient((N+1)*nx+ N*nu + (N+1) + i) = (terminal_CSS[i].cost - lowest_cost1) * q_time;
    }
    // for (int i=K_NEAR; i<2*K_NEAR; i++){
    //     gradient((N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].cost-lowest_cost2;
    // }
    for (int i=K_NEAR; i<2*K_NEAR; i++){
        gradient((N+1)*nx+ N*nu + (N+1) + i) = (terminal_CSS[i].cost - lowest_cost2) * q_time;
    }

    //

    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR) = q_s_terminal;   // x
    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 1, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 1) = q_s_terminal;   //y
    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 2, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 2) = q_s_terminal;   //yaw
    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 3, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 3) = q_s_terminal;   //v
    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 4, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 4) = q_s_terminal;   //r
    // HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 5, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 5) = q_s_terminal;   //beta

    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR) = q_s_terminal / (10.0);   // x
    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 1, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 1) = q_s_terminal / (10.0);   //y
    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 2, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 2) = q_s_terminal / 2.0;   //yaw
    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 3, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 3) = q_s_terminal / 10.0;   //v
    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 4, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 4) = q_s_terminal * 3.0;   //r
    HessianMatrix.insert((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 5, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + 5) = q_s_terminal * 2500.0;   //beta
//    for (int i=0; i<2*K_NEAR; i++){
//        gradient((N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].cost;
//    }

    // cout<<"gradient: "<<gradient<<endl;
    // cout<<"lower: "<<lower(517)<<endl;
    // cout<<"upper: "<<upper(517)<<endl;
    //x0 constraint
    lower.head(nx) = -x0;
    upper.head(nx) = -x0;

    SparseMatrix<double> H_t = HessianMatrix.transpose();
    SparseMatrix<double> sparse_I((N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);
    sparse_I.setIdentity();
    HessianMatrix = 0.5*(HessianMatrix + H_t) + 0.0000001*sparse_I;
    // ROS_INFO("HEREN");
    // cout << constraintMatrix<< endl;
    // cout << HessianMatrix<< endl;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables((N+1)*nx+ N*nu + (N+1)+ 2*K_NEAR +terminal_nx);
    solver.data()->setNumberOfConstraints((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx);

    if (!solver.data()->setHessianMatrix(HessianMatrix)) throw "fail set Hessian";
    if (!solver.data()->setGradient(gradient)){throw "fail to set gradient";}
    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix)) throw"fail to set constraint matrix";
    if (!solver.data()->setLowerBound(lower)){throw "fail to set lower bound";}
    if (!solver.data()->setUpperBound(upper)){throw "fail to set upper bound";}

    if(!solver.initSolver()){ cout<< "fail to initialize solver"<<endl;}

    if(!solver.solve()) {
        cout<< "fail to solve problem"<<endl;
        ros::shutdown();
        status_ = false;
        return;
    }
    QPSolution_ = solver.getSolution();
    visualize_mpc_solution(terminal_CSS, terminal_candidate);  // rviz可视化

   cout<<"Solution lamd: "<<endl;
    for (int index_lamd = 0; index_lamd < 2*K_NEAR; ++index_lamd){
        cout<<QPSolution_((N+1)*nx + N* nu + N+1 + index_lamd)<< ",";
    }
    cout << endl;
    cout<<"Solution s_k: "<<endl;
    for (int index_sk = 0; index_sk < N + 1; ++index_sk){
        cout<<QPSolution_((N+1)*nx + N* nu + index_sk)<< ",";
    }
    cout << endl;
    cout<<"Solution s_t: "<<endl;
    for (int index_st = 0; index_st < terminal_nx; ++index_st){
        cout<<QPSolution_((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + index_st)<< ",";
    }
    cout << endl;
    for (int index_v = 0; index_v < N + 1; ++index_v){
        cout<< "QPSOLUTION v = " << QPSolution_(index_v * nx + 3) << " QPSOLUTION r = " << QPSolution_(index_v * nx + 4) << " QPSOLUTION beta = " << QPSolution_(index_v * nx + 5) << " vx = " << QPSolution_(index_v * nx + 3)*cos(QPSolution_(index_v * nx + 5));
        if (index_v < N){
             cout << " QPSOLUTION Tf = " << QPSolution_((N+1) * nx + index_v * nu) <<  " QPSOLUTION delta = " << QPSolution_((N+1) * nx + index_v * nu + 2)<< endl;
        }
    }
    solver.clearSolver();

    if (use_dyn_) ROS_INFO("using dynamics");
    else ROS_INFO("using kinematics");
}

void LMPC::applyControl() {
    // cout<<"dert_Tf_cmd: "<<dert_Tf<<endl;
    // cout<<"dert_Tr_cmd: "<<dert_Tr<<endl;
    // cout << "dert_Steer_cmd: "<<dert_Steer<<endl;

    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = QPSolution_((N+1)*nx);
    cout<<"Tf_cmd: "<<  QPSolution_((N+1)*nx)<<endl;
    ack_msg.drive.acceleration = QPSolution_((N+1)*nx+1);
    cout<<"Tr_cmd: "<<QPSolution_((N+1)*nx+1)<<endl;
    ack_msg.drive.steering_angle =  QPSolution_((N+1)*nx+2) * 180.0 /M_PI;
    cout<<"Steer_cmd: "<<QPSolution_((N+1)*nx+2)<<endl;
    cout<<"Steer_cmd_deg: "<<QPSolution_((N+1)*nx+2) * 180.0 / M_PI<<endl;
    ack_msg.drive.steering_angle_velocity = 1.0;
    drive_pub_.publish(ack_msg);
    TF_LAST = QPSolution_((N+1)*nx);
    TR_LAST = QPSolution_((N+1)*nx+1);
    STEER_LAST = QPSolution_((N+1)*nx+2);
    last_vel_ = vel_;
    last_yawdot_ = yawdot_;
    last_slip_angle_ = slip_angle_;
}

void LMPC::record_status(){
    std_msgs::String pred_state;
    stringstream ss_x, ss_y, ss_v, ss_yaw, ss_r, ss_slip_angle, ss_Tf, ss_Tr, ss_delta,  ss_control_Tf, ss_control_Tr, ss_control_steer, ss_cur_x, ss_cur_y, ss_cur_yaw, ss_cur_v, ss_cur_r, ss_cur_slip_angle, ss_cur_Tf, ss_cur_Tr, ss_cur_delta, ss_vx_get, ss_vy_get, ss_vx_pre, ss_vy_pre, ss_error,ss_correct_v, ss_correct_r, ss_correct_beta;
    string s_state, s_x, s_y, s_v, s_yaw, s_r, s_slip_angle, s_Tf, s_Tr, s_delta,  s_control_Tf, s_control_Tr, s_control_steer, s_cur_x, s_cur_y, s_cur_yaw, s_cur_v, s_cur_r, s_cur_slip_angle, s_cur_Tf, s_cur_Tr, s_cur_delta, s_vx_get, s_vy_get, s_vx_pre, s_vy_pre, s_error, s_correct_v, s_correct_r, s_correct_beta;
    string comma = ",";
    ss_x << QPSolution_(nx);
    ss_y << QPSolution_(nx + 1);
    ss_yaw << QPSolution_(nx + 2);
    ss_v << QPSolution_(nx + 3);
    ss_r << QPSolution_(nx + 4);
    ss_slip_angle << QPSolution_(nx + 5);
    ss_Tf << QPSolution_((N+1)*nx);
    ss_Tr << QPSolution_((N+1)*nx + 1);
    ss_delta << QPSolution_((N+1)*nx + 2);
    ss_cur_x << QPSolution_(0);
    ss_cur_y << QPSolution_(1);
    ss_cur_yaw << QPSolution_(2);
    ss_cur_v << QPSolution_(3);
    ss_cur_r << QPSolution_(4);
    ss_cur_slip_angle << QPSolution_(5);
    ss_cur_Tf << Tf_;
    ss_cur_Tr << Tr_;
    ss_cur_delta << delta_;
    ss_vx_get << vx_;
    ss_vy_get <<  vy_;
    ss_vx_pre << QPSolution_(nx + 3) * cos(QPSolution_(nx + 5));
    ss_vy_pre << QPSolution_(nx + 3) * sin(QPSolution_(nx + 5));
    ss_error << error_num_;
    ss_correct_v << correct_v_0;
    ss_correct_r << correct_r_0;
    ss_correct_beta << correct_beta_0;

    ss_x >> s_x;
    ss_y >> s_y;
    ss_yaw >> s_yaw;
    ss_v >> s_v;
    ss_r >> s_r;
    ss_slip_angle >> s_slip_angle;
    ss_Tf >> s_Tf;
    ss_Tr >> s_Tr;
    ss_delta >> s_delta;
    ss_cur_x >> s_cur_x;
    ss_cur_y >> s_cur_y;
    ss_cur_yaw >> s_cur_yaw;
    ss_cur_v >> s_cur_v;
    ss_cur_r >> s_cur_r;
    ss_cur_slip_angle >> s_cur_slip_angle;
    ss_cur_Tf >> s_cur_Tf;
    ss_cur_Tr >> s_cur_Tr;
    ss_cur_delta >> s_cur_delta;
    ss_vx_get >> s_vx_get;
    ss_vy_get >> s_vy_get;
    ss_vx_pre >> s_vx_pre;
    ss_vy_pre >> s_vy_pre;
    ss_error >> s_error;
    ss_correct_v >> s_correct_v;
    ss_correct_r >> s_correct_r;
    ss_correct_beta >> s_correct_beta;

    s_state = s_x+ comma+s_y+comma+ s_yaw + comma + s_cur_x + comma + s_cur_y + comma + s_cur_yaw + " " + comma + s_v + comma + s_r + comma + s_slip_angle + comma + s_correct_v + comma + s_correct_r + comma + s_correct_beta + comma + s_cur_v + comma + s_cur_r + comma + s_cur_slip_angle + " " + comma+ s_Tf +comma+s_Tr + comma + s_delta + comma + s_cur_Tf +comma+ s_cur_Tr + comma + s_cur_delta +" " + comma +s_vx_pre + comma + s_vy_pre + comma + s_vx_get +comma + s_vy_get + " " + comma + s_error;
    pred_state.data = s_state;
    pred_pub_.publish(pred_state);
}

void LMPC::visualize_mpc_solution(const vector<Sample>& convex_safe_set, const Matrix<double,nx,1>& terminal_candidate) {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker pred_dots;
    pred_dots.header.stamp = ros::Time::now();
    pred_dots.header.frame_id = "map";
    pred_dots.id = rviz_id::PREDICTION;
    pred_dots.ns = "predicted_positions";
    pred_dots.type = visualization_msgs::Marker::POINTS;
    pred_dots.scale.x = pred_dots.scale.y = pred_dots.scale.z = 0.1;
    pred_dots.action = visualization_msgs::Marker::ADD;
    pred_dots.pose.orientation.w = 1.0;
    pred_dots.color.g = 1.0;
    pred_dots.color.a = 1.0;
    for (int i=0; i<N+1; i++){
        geometry_msgs::Point p;
        p.x = QPSolution_(i*nx);
        p.y = QPSolution_(i*nx+1);
        pred_dots.points.push_back(p);
    }
    markers.markers.push_back(pred_dots);

    visualization_msgs::Marker borderlines;
    borderlines.header.stamp = ros::Time::now();
    borderlines.header.frame_id = "map";
    borderlines.id = rviz_id::BORDERLINES;
    borderlines.ns = "borderlines";
    borderlines.type = visualization_msgs::Marker::LINE_LIST;
    borderlines.scale.x = 0.1;
    borderlines.action = visualization_msgs::Marker::ADD;
    borderlines.pose.orientation.w = 1.0;
    borderlines.color.r = 1.0;
    borderlines.color.a = 1.0;

    borderlines.points = border_lines_;
    markers.markers.push_back(borderlines);

    visualization_msgs::Marker css_dots;
    css_dots.header.stamp = ros::Time::now();
    css_dots.header.frame_id = "map";
    css_dots.id = rviz_id::SAFE_SET;
    css_dots.ns = "safe_set";
    css_dots.type = visualization_msgs::Marker::POINTS;
    css_dots.scale.x = css_dots.scale.y = css_dots.scale.z = 0.1;
    css_dots.action = visualization_msgs::Marker::ADD;
    css_dots.pose.orientation.w = 1.0;
    css_dots.color.g = 1.0;
    css_dots.color.b = 1.0;
    css_dots.color.a = 1.0;
    VectorXd costs = VectorXd(convex_safe_set.size());
    for (int i=0; i<convex_safe_set.size(); i++){
        geometry_msgs::Point p;
        p.x = convex_safe_set[i].x(0);
        p.y = convex_safe_set[i].x(1);
        css_dots.points.push_back(p);
        costs(i) = convex_safe_set[i].cost;
    }
    //cout<<"costs: "<<costs<< endl;
    markers.markers.push_back(css_dots);

    visualization_msgs::Marker terminal_dot;
    terminal_dot.header.stamp = ros::Time::now();
    terminal_dot.header.frame_id = "map";
    terminal_dot.id = rviz_id::TERMINAL_CANDIDATE;
    terminal_dot.ns = "terminal_candidate";
    terminal_dot.type = visualization_msgs::Marker::SPHERE;
    terminal_dot.scale.x = terminal_dot.scale.y = terminal_dot.scale.z = 0.1;
    terminal_dot.action = visualization_msgs::Marker::ADD;
    terminal_dot.pose.orientation.w = 1.0;
    terminal_dot.pose.position.x = terminal_candidate(0);
    terminal_dot.pose.position.y = terminal_candidate(1);
    terminal_dot.color.r = 0.5;
    terminal_dot.color.b = 0.8;
    terminal_dot.color.a = 1.0;
    markers.markers.push_back(terminal_dot);

    LMPC_viz_pub_.publish(markers);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "LMPC_nonlinear");
    ros::NodeHandle nh;
    LMPC lmpc(nh);
    ROS_INFO("Finish LMPC Initialization...");
    ros::Rate rate(20);
    // calculate time 
    double ts_start = 0.0;
    double ts_get_data = 0.0;
    double ts_calculate = 0.0;
    double ts_end = 0.0;
    while(ros::ok()){
        ts_start = clock();
        ros::spinOnce();
        ts_get_data = clock();
        // ROS_INFO("time_get_state = %f",(ts_get_data - ts_start)/CLOCKS_PER_SEC);
        if(marker_state == 0 or marker_odom == 0){
            continue;
        }
        ROS_INFO("start_run");
        lmpc.run();
        ts_calculate = clock();
        ROS_INFO("time_calculate = %f",(ts_calculate - ts_get_data)/CLOCKS_PER_SEC);
        rate.sleep();
        ts_end = clock();
        ROS_INFO("time = %f",(ts_end - ts_start)/CLOCKS_PER_SEC);
    }
    return 0;
}