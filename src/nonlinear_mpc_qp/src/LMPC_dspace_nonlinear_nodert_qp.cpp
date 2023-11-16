
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

#include <nonlinear_mpc_qp/track.h>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <nonlinear_mpc_qp/car_params.h>

const int nx = 8;   // [x, y, phi, v, r, beta, ax , ay]
const int nu = 3;   // [Tf, Tr, delta]
const int terminal_nx = 6;
const int RRT_INTERVAL = 0.1;
using namespace std;
using namespace qpOASES;
int marker_odom = 0;
int marker_state = 0;

struct Sample{
    Eigen::Matrix<double,nx,1> x;
    Eigen::Matrix<double,nu,1> u;
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
    double r_Tf;
    double r_Tr;
    double r_steer;
    Eigen::Matrix<double, nu, nu> R;

    Track* track_;
    //odometry
    tf::Transform tf_;
    tf::Vector3 car_pos_;
    double yaw_;
    double x_;
    double vel_;
    double yawdot_;
    double slip_angle_;
    double s_prev_;
    double s_curr_;
    double s_ref_cur_;
    double phi_;
    double Tf_;
    double Tr_;
    double delta_;
    double ax_;
    double ay_;
    ofstream lap_time_file_;
    int lap_;
    bool status_;
    int error_num_;

    // use dynamic model or not
    bool use_dyn_;

    //Sample Safe set
    vector<vector<Sample>> SS_;
    vector<Sample> curr_trajectory_;
    int iter_;
    int time_;
    Eigen::Matrix<double,nx,1> terminal_state_pred_;

    // map info
    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid map_updated_;
    vector<geometry_msgs::Point> rrt_path_;


    Eigen::VectorXd QPSolution_;
    bool first_run_ = true;
    bool first_lap_ = true;
    vector<geometry_msgs::Point> border_lines_;


    void getParameters(ros::NodeHandle& nh);
    void init_occupancy_grid();
    void init_SS_from_data(const string data_file);
    void map_callback(const nav_msgs::OccupancyGrid::Ptr &map_msg);
    void rrt_path_callback(const visualization_msgs::Marker::ConstPtr &path_msg);
    void visualize_centerline();
    void copy_Sparse_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols);
    void copy_Vector_to_real_t(real_t* target, Eigen::VectorXd& source, int length);
    void copy_real_t_to_Vector(Eigen::VectorXd& target, real_t* source,int length);
    int reset_QPSolution(int iter);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void state_callback(const sensor_msgs::Imu::ConstPtr &state_msg);
    void add_point();

    void solve_MPC(const Eigen::Matrix<double,nx,1>& terminal_candidate);

    void get_linearized_dynamics(Eigen::Matrix<double,nx,nx>& Ad, Eigen::Matrix<double,nx, nu>& Bd, Eigen::Matrix<double,nx,1>& hd,
            Eigen::Matrix<double,nx,1>& x_op, Eigen::Matrix<double,nu,1>& u_op, bool use_dyn);

    Eigen::Vector3d global_to_track(double x, double y, double yaw, double s);
    Eigen::Vector3d track_to_global(double e_y, double e_yaw, double s);

    void applyControl();
    void record_status();
    void visualize_mpc_solution(const vector<Sample>& convex_safe_set, const Eigen::Matrix<double,nx,1>& terminal_candidate);

    Eigen::Matrix<double,nx,1> select_terminal_candidate();
    void select_convex_safe_set(vector<Sample>& convex_safe_set, int iter_start, int iter_end, double s);
    int find_nearest_point(vector<Sample>& trajectory, double s);
    void update_cost_to_go(vector<Sample>& trajectory);
};

LMPC::LMPC(ros::NodeHandle &nh): nh_(nh){  // 构造函数

    getParameters(nh_);  // 从launch文件指引的lmpc_params.yaml提取参数
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
    init_SS_from_data(init_data_file);
    lap_time_file_.open("/home/sun234/racing_work/src/LearningMPC-master/data/lap_time_set_10.csv");
    lap_ = 0;
    error_num_ = 0;
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
        sample.x(0) = std::stof(vec.at(1));   // x
        sample.x(1) = std::stof(vec.at(2));  // y
        sample.s = std::stof(vec.at(3));  // s
        sample.x(2) = std::stof(vec.at(4));  // yaw
        sample.x(3) = std::stof(vec.at(5));  // v
        sample.x(4) = std::stof(vec.at(6));  // r
        sample.x(5) = std::stof(vec.at(7));  // beta
        sample.x(6) = std::stof(vec.at(11));  // Ax
        sample.x(7) = std::stof(vec.at(12));  //Ay
        sample.u(0) = std::stof(vec.at(13));  // dert Tf
        sample.u(1) = std::stof(vec.at(14));  // dert Tr
        sample.u(2) = std::stof(vec.at(15));  // dert Steer
        sample.iter = it;
        traj.push_back(sample);
        time_prev = sample.time;
    }
    update_cost_to_go(traj);
    SS_.push_back(traj);
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
        Eigen::Vector2d p_path(path_processed[i].x,path_processed[i].y);
        Eigen::Vector2d p_proj(track_->x_eval(theta), track_->y_eval(theta));
        Eigen::Vector2d p1, p2;
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
        double right_width = Eigen::Vector2d(dy_dtheta, -dx_dtheta).dot(p1-p_proj)>0 ? (p1-p_proj).norm() : -(p1-p_proj).norm();
        double left_width = Eigen::Vector2d(-dy_dtheta, dx_dtheta).dot(p2-p_proj)>0 ? (p2-p_proj).norm() : -(p2-p_proj).norm();
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
    s_curr_ = track_->findTheta(x, y,0,s_prev_,true);  // 当前状态的s
    car_pos_ = tf::Vector3(x, y, 0.0);
    x_ = x;
    yaw_ = tf::getYaw(odom_msg->pose.pose.orientation);
    vel_ = sqrt(pow(odom_msg->twist.twist.linear.x,2) + pow(odom_msg->twist.twist.linear.y,2));
    if (abs(vel_) < 1e-4){
        vel_ += 1e-4;
    }
    yawdot_ = odom_msg->twist.twist.angular.z;
    slip_angle_ = atan2(odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.x);
    cout << "slip_angle_ = " << slip_angle_ << endl;

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

    /*** select terminal state candidate and its convex safe set ***/
    Eigen::Matrix<double,nx,1> terminal_candidate = select_terminal_candidate();  // 选择z
    ROS_INFO("Terminal Candidate x = %lf, y = %lf, yaw = %lf, vel = %lf, r = %lf, beta = %lf, Ax = %lf, Ay = %lf", terminal_candidate(0), terminal_candidate(1),terminal_candidate(2), terminal_candidate(3), terminal_candidate(4), terminal_candidate(5), terminal_candidate(6), terminal_candidate(7));
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
    QPSolution_ = Eigen::VectorXd::Zero((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) + (terminal_nx));   // QPSolution (N+1)*nx[状态] + N * nu[控制] + nx * (N+1)[松弛] + 2 * (K_Near + 1)[权重]
    for (int i=0; i<N+1; i++){
        QPSolution_.segment<nx>(i*nx) = SS_[iter][i].x;
        if (i<N) QPSolution_.segment<nu>((N+1)*nx + i*nu) = SS_[iter][i].u;
    }
}

Eigen::Matrix<double,nx,1> LMPC::select_terminal_candidate(){
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
    point.x << car_pos_.x(), car_pos_.y(), yaw_, vel_,  yawdot_, slip_angle_, ax_, ay_;
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
                std::cout <<"ind = " << ind << "SS_[it][ind] x = " <<  SS_[it][ind].x[0] << " SS_[it][ind] y = "<< SS_[it][ind].x[1] << " SS_[it][ind] yaw = "<< SS_[it][ind].x[2] << " SS_[it][ind] v = " << SS_[it][ind].x[3] << " SS_[it][ind] cost = " << SS_[it][ind].cost << std::endl;
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

Eigen::Vector3d LMPC::global_to_track(double x, double y, double yaw, double s){
    double x_proj = track_->x_eval(s);
    double y_proj = track_->y_eval(s);
    double e_y = sqrt((x-x_proj)*(x-x_proj) + (y-y_proj)*(y-y_proj));
    double dx_ds = track_->x_eval_d(s);
    double dy_ds = track_->y_eval_d(s);
    e_y = dx_ds*(y-y_proj) - dy_ds*(x-x_proj) >0 ? e_y : -e_y;
    double e_yaw = yaw - atan2(dy_ds, dx_ds);
    while(e_yaw > M_PI) e_yaw -= 2*M_PI;
    while(e_yaw < -M_PI) e_yaw += 2*M_PI;

    return Eigen::Vector3d(e_y, e_yaw, s);
}

Eigen::Vector3d LMPC::track_to_global(double e_y, double e_yaw, double s){
    double dx_ds = track_->x_eval_d(s);
    double dy_ds = track_->y_eval_d(s);
    Eigen::Vector2d proj(track_->x_eval(s), track_->y_eval(s));
    Eigen::Vector2d pos = proj + Eigen::Vector2d(-dy_ds, dx_ds).normalized()*e_y;
    double yaw = e_yaw + atan2(dy_ds, dx_ds);
    return Eigen::Vector3d(pos(0), pos(1), yaw);
}

void LMPC::get_linearized_dynamics(Eigen::Matrix<double,nx,nx>& Ad, Eigen::Matrix<double,nx, nu>& Bd, Eigen::Matrix<double,nx,1>& hd,
        Eigen::Matrix<double,nx,1>& x_op, Eigen::Matrix<double,nu,1>& u_op, bool use_dyn){
    
    double yaw = x_op(2);
    double v = x_op(3);
    if (abs(v - 0.0) < 1e-4){
        v += 0.0001;
    }
    double yaw_dot = x_op(4);
    double slip_angle = x_op(5);
    double ax = x_op(6);
    double ay = x_op(7);
    double T_f = u_op(0);
    double T_r = u_op(1);
    double delta = u_op(2);

    Eigen::VectorXd dynamics(8), h(8);
    Eigen::Matrix<double, nx, nx> A, M12;
    Eigen::Matrix<double, nx, nu> B;

    // Single Track Dynamic Model

    double g = 9.81;

    dynamics(0) = v * cos(yaw+slip_angle);
    dynamics(1) = v * sin(yaw+slip_angle);
    dynamics(2) = yaw_dot;
    double Fzf = car.mass * (g * car.l_r - ax * car.h_cg) / car.wheelbase;
    double Fzr = car.mass * (g * car.l_f + ax * car.h_cg) / car.wheelbase;
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
    dynamics(6) = (Fxf * cos(delta) - Fyf * sin(delta) + Fxr) / car.mass;
    dynamics(7) = (Fxf * sin(delta) + Fyf * cos(delta) + Fyr) / car.mass;

    cout << "dynamic = " << dynamics << endl;

    double dfx_dyaw, dfx_dv, dfx_dslip, dfy_dyaw, dfy_dv, dfy_dslip, dfv_dv, dfv_dyawdot, 
        dfv_dslip, dfv_dTf, dfv_dTr, dfv_ddelta, dfv_dax, dfyawdot_dv, dfyawdot_dyawdot,dfyawdot_dslip,  dfyawdot_dTf, dfyawdot_ddelta, dfyawdot_dax, dfslip_dv, 
        dfslip_dyawdot, dfslip_dslip, dfslip_dTf, dfslip_dTr, dfslip_ddelta, dfslip_dax, dfax_dv, dfax_dyawdot, dfax_dslip, dfax_dTf, dfax_dTr, dfax_ddelta, dfax_dax, 
        dfay_dv, dfay_dyawdot, dfay_dslip, dfay_dTf, dfay_ddelta, dfay_dax;
    double dfyawdot_da, dfyawdot_dsteer, dfslip_da, dfslip_dsteer;

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
    double t5 = sin(delta);
    double t6 = car.l_f * g;
    double t7 = car.l_r * g;
    double t8 = car.l_f * yaw_dot;
    double t9 = car.l_r * yaw_dot;
    double t10 = ax * car.h_cg;
    double t11 = slip_angle + yaw;
    double t13 = pow(car.Bf, 2);
    double t14 = pow(car.Br, 2);
    double t17 = 1.0 / car.I_z;
    double t19 = 1.0 / car.wheelbase;
    double t20 = 1.0 / car.R_wheel;
    double t21 = -delta;
    double t22 = 1.0 / car.mass;
    double t23 = 1.0 / v;
    double t15 = cos(t11);
    double t16 = sin(t11);
    double t18 = t4 * v;
    double t24 = pow(t23,2);
    double t25 = 1.0 / t2;
    double t27 = -t10;
    double t29 = slip_angle+t21;
    double t32 = t6+t10;
    double t36 = T_f * t3 * t20;
    double t37 = T_r * t4 * t20;
    double t26 = pow(t25, 2);
    double t28 = t15 * v;
    double t31 = -t18;
    double t33 = cos(t29);
    double t34 = sin(t29);
    double t35 = t8+t18;
    double t38 = t7+t27;
    double t43 = t4 *t23 *t25;
    double t41 = t9+t31;
    double t42 = pow(t35, 2);
    double t45 = T_f *t20 *t33;
    double t46 = T_f *t20 *t34;
    double t49 = t23 *t25 *t35;
    double t50 = t24 *t25 *t35;
    double t53 = t4 *t23 *t26 *t35;
    double t44 = pow(t41, 2);
    double t52 = atan(t49);
    double t54 = -t50;
    double t55 = t23 *t25 *t41;
    double t56 = t24 *t25 *t41;
    double t57 = t24 *t26 *t42;
    double t61 = t4 *t23 *t26 *t41;
    double t62 = t53+1.0;
    double t58 = atan(t55);
    double t59 = t57+1.0;
    double t60 = -t52;
    double t63 = t24 *t26 *t44;
    double t71 = t61-1.0;
    double t82 = t43+t54;
    double t83 = t43+t56;
    double t64 = pow(t58, 2);
    double t65 = car.Br *t58;
    double t66 = delta+t60;
    double t67 = t63+1.0;
    double t70 = 1.0 /t59;
    double t68 = atan(t65);
    double t69 = -t65;
    double t72 = pow(t66, 2);
    double t73 = car.Bf *t66;
    double t75 = t14 *t64;
    double t76 = 1.0/t67;
    double t85 = car.Bf *car.l_f *t23 *t25 *t70;
    double t95 = car.Bf *t62 *t70;
    double t103 = car.Bf *t70 *t82;
    double t74 = atan(t73);
    double t77 = -t73;
    double t78 = t75+1.0;
    double t79 = t13 *t72;
    double t87 = -t85;
    double t88 = car.Br *car.l_r *t23 *t25 *t76;
    double t96 = t68+t69;
    double t97 = -t95;
    double t98 = -car.Er *(t65-t68);
    double t99 = car.Br *t71 *t76;
    double t104 = -t103;
    double t105 = car.Br *t76 *t83;
    double t80 = 1.0 /t78;
    double t81 = t79+1.0;
    double t91 = -t88;
    double t100 = t74+t77;
    double t101 = -t99;
    double t102 = -car.Ef *(t73-t74);
    double t106 = -t105;
    double t111 = t65+t98;
    double t84 = 1.0 /t81;
    double t107 = t80 *t88;
    double t109 = t80 *t91;
    double t112 = atan(t111);
    double t113 = pow(t111, 2);
    double t120 = t73+t102;
    double t123 = t80 *t99;
    double t126 = t80 *t101;
    double t131 = t80 *t105;
    double t134 = t80 *t106;
    double t86 = car.Bf *t84;
    double t108 = t84 *t85;
    double t110 = t84 *t87;
    double t114 = car.Cr *t112;
    double t115 = t113+1.0;
    double t122 = atan(t120);
    double t124 = pow(t120, 2);
    double t142 = t88+t109;
    double t148 = t99+t126;
    double t153 = t105+t134;
    double t89 = -t86;
    double t116 = cos(t114);
    double t117 = sin(t114);
    double t118 = 1.0 /t115;
    double t119 = t62 *t70 *t86;
    double t125 = car.Cf *t122;
    double t129 = t124+1.0;
    double t132 = t70 *t82 *t86;
    double t139 = t85+t110;
    double t145 = car.Er *t142;
    double t149 = car.Er *t148;
    double t155 = car.Er *t153;
    double t90 = car.Bf+t89;
    double t121 = t62 *t70 *t89;
    double t127 = cos(t125);
    double t128 = sin(t125);
    double t130 = 1.0 /t129;
    double t133 = car.mass *car.friction_coeff *t2 *t19 *t32 *t117;
    double t135 = t70 *t82 *t89;
    double t143 = car.Ef *t139;
    double t151 = t91+t145;
    double t157 = t101+t149;
    double t159 = t106+t155;
    double t92 = car.Ef *t90;
    double t136 = -t133;
    double t137 = car.mass *car.friction_coeff*t5 *t19 *t38 *t128;
    double t140 = car.mass *car.friction_coeff*t19 *t33 *t38 *t128;
    double t141 = car.mass *car.friction_coeff *t19 *t34 *t38 *t128;
    double t146 = t95+t121;
    double t150 = t87+t143;
    double t152 = t103+t135;
    double t93 = -t92;
    double t138 = -t137;
    double t144 = -t140;
    double t147 = car.Ef *t146;
    double t154 = car.Ef *t152;
    double t94 = car.Bf+t93;
    double t156 = t97+t147;
    double t158 = t104+t154;
    double t160 = car.Cf *car.mass *car.friction_coeff*t3 *t19 *t38 *t94 *t127 *t130;
    double t161 = t36+t138+t160;

    dfv_dv = -t22 * (car.Cf *car.mass *car.friction_coeff *t19 *t34 *t38 *t127*t130 *(t103-t154)+car.Cr *car.mass *car.friction_coeff *t4 *t19 *t32 *t116 *t118 *(t105-t155));
    dfv_dyawdot = -t22 *(car.Cf *car.mass *car.friction_coeff *t19 *t34 *t38 *t127 *t130 *(t85-t143)-car.Cr *car.mass *car.friction_coeff*t4 *t19 *t32 *t116 *t118 *(t88-t145));
    dfv_dslip = -t22 *(t37+t46+t136+t144+car.Cf *car.mass *car.friction_coeff *t19 *t34 *t38 *t127 *t130 *(t95-t147)-car.Cr *car.mass *car.friction_coeff *t4 *t19 *t32 *t116 *t118 *(t99-t149));
    dfv_dTf = t20 *t22 *t33;
    dfv_dTr = t2 *t20 *t22;
    dfv_ddelta = t22 *(t46+t144+car.Cf *car.mass *car.friction_coeff *t19 *t34 *t38 *t94 *t127 *t130);
    dfv_dax = t22 *(car.h_cg *car.mass *car.friction_coeff *t4 *t19 *t117-car.h_cg *car.mass *car.friction_coeff *t19 *t34 *t128);

    // ROS_INFO("dfv_dv = %lf", dfv_dv);
    // ROS_INFO("dfv_dyawdot = %lf", dfv_dyawdot);
    // ROS_INFO("dfv_dslip = %lf", dfv_dslip);
    // ROS_INFO("dfv_dTf = %lf", dfv_dTf);
    // ROS_INFO("dfv_dTr = %lf", dfv_dTr);
    // ROS_INFO("dfv_ddelta = %lf", dfv_ddelta);
    // ROS_INFO("dfv_dax = %lf", dfv_dax);

    dfyawdot_dv = t17 *(car.Cr *car.l_r *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t105-t155)-car.Cf *car.l_f *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t103-t154));
    dfyawdot_dyawdot = -t17 *(car.Cr *car.l_r *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t88-t145)+car.Cf *car.l_f *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t85-t143));
    dfyawdot_dslip = -t17 *(car.Cr *car.l_r *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t99-t149)+car.Cf *car.l_f *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t95-t147));
    dfyawdot_dTf = car.l_f *t5 *t17 *t20;
    dfyawdot_ddelta = car.l_f *t17 *t161;
    dfyawdot_dax = -t17 *(car.l_r *car.h_cg *car.mass *car.friction_coeff *t19 *t117+car.l_f *car.h_cg *car.mass *car.friction_coeff *t3 *t19 *t128);

    // ROS_INFO("dfyawdot_dv = %lf", dfyawdot_dv);
    // ROS_INFO("dfyawdot_dyawdot = %lf", dfyawdot_dyawdot);
    // ROS_INFO("dfyawdot_dslip = %lf", dfyawdot_dslip);
    // ROS_INFO("dfyawdot_dTf = %lf", dfyawdot_dTf);
    // ROS_INFO("dfyawdot_ddelta = %lf", dfyawdot_ddelta);
    // ROS_INFO("dfyawdot_dax = %lf", dfyawdot_dax);

    dfslip_dv = t22 *t24 *(t37+t46+t136+t144)-t22 *t23 *(car.Cf *car.mass *car.friction_coeff *t19 *t33 *t38 *t127 *t130 *(t103-t154)+car.Cr *car.mass *car.friction_coeff *t2 *t19 *t32 *t116 *t118 *(t105-t155));
    dfslip_dyawdot = -t22 *t23 *(car.Cf *car.mass *car.friction_coeff *t19 *t33 *t38 *t127 *t130 *(t85-t143)-car.Cr *car.mass *car.friction_coeff *t2 *t19 *t32 *t116 *t118 *(t88-t145))-1.0;
    dfslip_dslip = -t22 *t23 *(t45+t141+T_r *t2 *t20+car.mass *car.friction_coeff *t4 *t19 *t32 *t117+car.Cf *car.mass *car.friction_coeff *t19 *t33 *t38 *t127 *t130 *(t95-t147)-car.Cr *car.mass *car.friction_coeff *t2 *t19 *t32 *t116 *t118 *(t99-t149));
    dfslip_dTf = -t20 *t22 *t23 *t34;
    dfslip_dTr = -t4 *t20 *t22 *t23;
    dfslip_ddelta = t22 *t23 *(t45+t141+car.Cf *car.mass *car.friction_coeff *t19 *t33 *t38 *t94 *t127 *t130);
    dfslip_dax = t22 *t23 *(car.h_cg *car.mass *car.friction_coeff *t2 *t19 *t117-car.h_cg *car.mass *car.friction_coeff *t19 *t33 *t128);

    // ROS_INFO("dfslip_dv = %lf", dfslip_dv);
    // ROS_INFO("dfslip_dyawdot = %lf", dfslip_dyawdot);
    // ROS_INFO("dfslip_dslip = %lf", dfslip_dslip);
    // ROS_INFO("dfslip_dTf = %lf", dfslip_dTf);
    // ROS_INFO("dfslip_dTr = %lf", dfslip_dTr);
    // ROS_INFO("dfslip_ddelta = %lf", dfslip_ddelta);
    // ROS_INFO("dfslip_dax = %lf", dfslip_dax);

    dfax_dv = car.Cf *car.friction_coeff *t5 *t19 *t38 *t127 *t130 *(t103-t154);
    dfax_dyawdot = car.Cf *car.friction_coeff *t5 *t19 *t38 *t127 *t130 *(t85-t143);
    dfax_dslip = car.Cf *car.friction_coeff *t5 *t19 *t38 *t127 *t130 *(t95-t147);
    dfax_dTf = t3 *t20 *t22;
    dfax_dTr = t20 *t22;
    dfax_ddelta = -t22 *(T_f *t5 *t20+car.mass *car.friction_coeff *t3 *t19 *t38 *t128+car.Cf *car.mass *car.friction_coeff *t5 *t19 *t38 *t94 *t127 *t130);
    dfax_dax = car.h_cg *car.friction_coeff *t5 *t19 *t128;

    // ROS_INFO("dfax_dv = %lf", dfax_dv);
    // ROS_INFO("dfax_dyawdot = %lf", dfax_dyawdot);
    // ROS_INFO("dfax_dslip = %lf", dfax_dslip);
    // ROS_INFO("dfax_dTf = %lf", dfax_dTf);
    // ROS_INFO("dfax_dTr = %lf", dfax_dTr);
    // ROS_INFO("dfax_ddelta = %lf", dfax_ddelta);
    // ROS_INFO("dfax_dax = %lf", dfax_dax);

    dfay_dv = -t22 *(car.Cr *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t105-t155)+car.Cf *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t103-t154));
    dfay_dyawdot = t22 *(car.Cr *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t88-t145)-car.Cf *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t85-t143));
    dfay_dslip = t22 *(car.Cr *car.mass *car.friction_coeff *t19 *t32 *t116 *t118 *(t99-t149)-car.Cf *car.mass *car.friction_coeff *t3 *t19 *t38 *t127 *t130 *(t95-t147));
    dfay_dTf = t5 *t20 *t22;
    dfay_ddelta = t22 *t161;
    dfay_dax = t22 *(car.h_cg *car.mass *car.friction_coeff *t19 *t117-car.h_cg *car.mass *car.friction_coeff *t3 *t19 *t128);

    // ROS_INFO("dfay_dv = %lf", dfay_dv);
    // ROS_INFO("dfay_dyawdot = %lf", dfay_dyawdot);
    // ROS_INFO("dfay_dslip = %lf", dfay_dslip);
    // ROS_INFO("dfay_dTf = %lf", dfay_dTf);
    // ROS_INFO("dfay_ddelta = %lf", dfay_ddelta);
    // ROS_INFO("dfay_dax = %lf", dfay_dax);


    A <<    0.0, 0.0, dfx_dyaw, dfx_dv, 0.0,  dfx_dslip, 0.0, 0.0,
            0.0,           0.0,    dfy_dyaw, dfy_dv,0.0,  dfy_dslip, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0,           0.0,               0.0,           dfv_dv, dfv_dyawdot, dfv_dslip, dfv_dax, 0.0,
            0.0,           0.0,               0.0,           dfyawdot_dv,     dfyawdot_dyawdot,    dfyawdot_dslip, dfyawdot_dax, 0.0,
            0.0,           0.0,               0.0,             dfslip_dv,       dfslip_dyawdot,             dfslip_dslip, dfslip_dax, 0.0,
            0.0,           0.0,               0.0,              0.0,    0.0,        0.0,        0.0,        0.0,
            0.0,           0.0,               0.0,              0.0,    0.0,        0.0,        0.0,        0.0;

    B <<    0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            dfv_dTf, dfv_dTr, dfv_ddelta,
            dfyawdot_dTf, 0.0, dfyawdot_ddelta,
            dfslip_dTf,   dfslip_dTr,   dfslip_ddelta,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;

    /**  Discretize using Zero-Order Hold **/
    // Eigen::Matrix<double,nx+nx,nx+nx> aux, M;
    // aux.setZero();
    // aux.block<nx,nx>(0,0) << A;
    // aux.block<nx,nx>(0, nx) << Eigen::Matrix<double,nx,nx>::Identity();
    // M = (aux*Ts).exp();
    // M12 = M.block<nx,nx>(0,nx);
    // h = dynamics - (A*x_op + B*u_op);

    // Eigen::Matrix<double, nx, nx> Ad_init = (A*Ts).exp();
    // Eigen::Matrix<double, nx, nu> Bd_init = M12*B;
    // Eigen::Matrix<double, nx, 1> hd_init = M12*h;

    Eigen::Matrix<double, nx, nx> Ad_init = A * Ts + Eigen::Matrix<double, nx, nx>::Identity();
    Eigen::Matrix<double, nx, nu> Bd_init = B * Ts;
    Eigen::Matrix<double, nx, 1> hd_init = (dynamics - (A * x_op + B * u_op))*Ts;

    Eigen::Matrix<double, 2, nx> A_down;
    Eigen::Matrix<double, 2, nu> B_down;
    A_down << 0.0,     0.0,        0.0,  dfax_dv,    dfax_dyawdot,  dfax_dslip,    dfax_dax,  0.0,
        0.0,  0.0,   0.0,     dfay_dv,    dfay_dyawdot,    dfay_dslip,     dfay_dax,    0.0;
    B_down << dfax_dTf,  dfax_dTr,   dfax_ddelta,
        dfay_dTf,        0.0,  dfay_ddelta;
    
    Ad.block<nx - 2, nx>(0,0) << Ad_init.block<nx - 2, nx>(0, 0);
    Ad.block<2, nx>(nx -2 , 0) << A_down;
    Bd.block<nx- 2, nu>(0,0) << Bd_init.block<nx-2, nu>(0,0);
    Bd.block<2,nu>(nx-2, 0) << B_down;
    hd.block<nx - 2, 1>(0,0) << hd_init.block<nx- 2, 1>(0,0);
    hd.block<2, 1>(nx-2, 0) << (dynamics.segment<2>(nx - 2) - A_down * x_op - B_down * u_op);
    // cout << "A = " << A << endl;
    // cout << "B = " << B << endl;
    // cout << "Ad_init = " << Ad_init << endl;
    // cout << "Bd_init = " << Bd_init << endl;
    // cout << "hd_init = " << hd_init << endl;
    // cout << "A_down = " << A_down << endl;
    // cout << "B_down = " << B_down << endl;
    // cout << "Ad = " << Ad << endl;
    // cout << "Bd = " << Bd << endl;
    // cout << "hd = " << hd << endl;
}

void wrap_angle(double& angle, const double angle_ref){
    while(angle - angle_ref > M_PI) {angle -= 2*M_PI;}
    while(angle - angle_ref < -M_PI) {angle += 2*M_PI;}
}

void LMPC::solve_MPC(const Eigen::Matrix<double,nx,1>& terminal_candidate){
    vector<Sample> terminal_CSS;  // 终端凸区间
    double s_t = track_->findTheta(terminal_candidate(0), terminal_candidate(1), 0, true);   // 找z的s
    select_convex_safe_set(terminal_CSS, iter_-2, iter_-1, s_t);  // 在前两圈中根据z选择terminal_CSS
    ROS_INFO("HERE");
    /** MPC variables: z = [x0, ..., xN, u0, ..., uN-1, s0, ..., sN, lambda0, ....., lambda(2*K_NEAR), s_t1, s_t2, .. s_t6]*
    *  constraints: dynamics, track bounds, input limits, acceleration limit, slack, lambdas, terminal state, sum of lambda's*/
    int Hessian_row = ((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx);
    int Hessian_col = ((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx);
    int ConstraintA_row = (N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx;
    int ConstraintA_col = ((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx);
    int ConstraintA2_row = 0;
    real_t HHessianMatrix[Hessian_row * Hessian_col];
    real_t GMatrix[Hessian_row];
    for (int index_G = 0; index_G < Hessian_row; ++index_G){
        GMatrix[index_G] = 0.0;
    }
    real_t ConstraintAMatrix[ConstraintA_row * ConstraintA_col];
    real_t lbA[ConstraintA_row];
    real_t ubA[ConstraintA_row];
    real_t lb[ConstraintA2_row];
    real_t ub[ConstraintA2_row];
    Eigen::MatrixXd HessianMatrix((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);
    Eigen::MatrixXd constraintMatrix((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);

    Eigen::VectorXd gradient((N+1)*nx+ N*nu + (N+1) + (2*K_NEAR) +terminal_nx);

    Eigen::VectorXd lower((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx);
    Eigen::VectorXd upper((N+1)*nx+ 2*(N+1) + N*nu + 3*(N+1) + (N+1) + 3 * N + (2*K_NEAR) + 2*terminal_nx+1 + terminal_nx);

    gradient.setZero();
    lower.setZero(); upper.setZero();

    Eigen::Matrix<double,nx,1> x_k_ref;
    Eigen::Matrix<double,nu,1> u_k_ref;
    Eigen::Matrix<double,nx,nx> Ad;
    Eigen::Matrix<double,nx,nu> Bd;
    Eigen::Matrix<double,nx,1> x0, hd;
    border_lines_.clear();

    x0 << car_pos_.x(), car_pos_.y(),  yaw_, vel_, yawdot_, slip_angle_, ax_, ay_;
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
        ROS_INFO("S_ref = %lf, X_ref = %lf, Y_ref = %lf, Yaw_ref = %lf, Vel_ref = %lf, Yaw_rate_ref = %lf, Beta_ref = %lf, Ax_ref = %lf , Ay_ref = %lf", s_ref, x_k_ref(0),  x_k_ref(1),  x_k_ref(2),  x_k_ref(3),  x_k_ref(4),  x_k_ref(5),  x_k_ref(6),  x_k_ref(7));
        s_ref_cur_ = s_ref;
        get_linearized_dynamics(Ad, Bd, hd, x_k_ref, u_k_ref, use_dyn_);   // 计算0阶离散线性系统矩阵Ad, Bd, hd
        ROS_INFO("HEREdone");
        /* form Hessian entries*/
        // cost does not depend on x0, only 1 to N
        if (i > 0){
            HessianMatrix((N+1)*nx + N*nu + i, (N+1)*nx + N*nu + i) = q_s;
        }
        if (i<N){
            for (int row=0; row<nu; row++){
                HessianMatrix((N+1)*nx + i*nu + row, (N+1)*nx + i*nu + row) = R(row, row);
            }
        }

        /* form constraint matrix */
        if (i<N){
            // Ad
            for (int row=0; row<nx; row++){
                for(int col=0; col<nx; col++){
                    constraintMatrix((i+1)*nx+row, i*nx+col) = Ad(row,col);
                }
            }
            // Bd
            for (int row=0; row<nx; row++){
                for(int col=0; col<nu; col++){
                    constraintMatrix((i+1)*nx+row, (N+1)*nx+ i*nu+col) = Bd(row,col);
                }
            }
            lower.segment<nx>((i+1)*nx) = -hd;//-OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY,-OsqpEigen::INFTY;//-hd;
            upper.segment<nx>((i+1)*nx) = -hd; //OsqpEigen::INFTY, OsqpEigen::INFTY,OsqpEigen::INFTY,OsqpEigen::INFTY;//-hd;
        }

        // -I for each x_k+1
        for (int row=0; row<nx; row++) {
            constraintMatrix(i*nx+row, i*nx+row) = -1.0;
        }

        double dx_dtheta = track_->x_eval_d(s_ref);
        double dy_dtheta = track_->y_eval_d(s_ref);

        constraintMatrix((N+1)*nx+ 2*i, i*nx) = -dy_dtheta;      // a*x
        constraintMatrix((N+1)*nx+ 2*i, i*nx+1) = dx_dtheta;     // b*y
        constraintMatrix((N+1)*nx+ 2*i, (N+1)*nx +N*nu +i) = 1.0;   // min(C1,C2) <= a*x + b*y + s_k <= inf

        constraintMatrix((N+1)*nx+ 2*i+1, i*nx) = -dy_dtheta;      // a*x
        constraintMatrix((N+1)*nx+ 2*i+1, i*nx+1) = dx_dtheta;     // b*y
        constraintMatrix((N+1)*nx+ 2*i+1, (N+1)*nx +N*nu +i) = -1.0;   // -inf <= a*x + b*y - s_k <= max(C1,C2)
        cout << "dy_dtheta = " << dy_dtheta << ", " << "dx_dtheta = " << dx_dtheta << endl;

        //get upper line and lower line
        Eigen::Vector2d left_tangent_p, right_tangent_p, center_p;
        Eigen::Vector2d right_line_p1, right_line_p2, left_line_p1, left_line_p2;
        geometry_msgs::Point r_p1, r_p2, l_p1, l_p2;

        center_p << track_->x_eval(s_ref), track_->y_eval(s_ref);
        right_tangent_p = center_p + track_->getRightHalfWidth(s_ref) * Eigen::Vector2d(dy_dtheta, -dx_dtheta).normalized();   // centerp投影点
        left_tangent_p  = center_p + track_->getLeftHalfWidth(s_ref) * Eigen::Vector2d(-dy_dtheta, dx_dtheta).normalized();

        right_line_p1 = right_tangent_p + 0.15*Eigen::Vector2d(dx_dtheta, dy_dtheta).normalized();
        right_line_p2 = right_tangent_p - 0.15*Eigen::Vector2d(dx_dtheta, dy_dtheta).normalized();
        left_line_p1 = left_tangent_p + 0.15*Eigen::Vector2d(dx_dtheta, dy_dtheta).normalized();
        left_line_p2 = left_tangent_p - 0.15*Eigen::Vector2d(dx_dtheta, dy_dtheta).normalized();

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
        upper((N+1)*nx+ 2*i) = INFINITY;

        lower((N+1)*nx+ 2*i+1) = -INFINITY;
        upper((N+1)*nx+ 2*i+1) = max(C1, C2);
        std::cout << "max(C1, C2) = " << max(C1, C2) << std::endl;

        // u_min < u < u_max
        if (i<N){
            for (int row=0; row<nu; row++){
                constraintMatrix((N+1)*nx+ 2*(N+1) +i*nu+row, (N+1)*nx+i*nu+row) = 1.0;
            }
            // input bounds: speed and steer
            lower.segment<nu>((N+1)*nx+ 2*(N+1) +i*nu) <<  - TF_MAX, -TR_MAX, -STEER_MAX;
            upper.segment<nu>((N+1)*nx+ 2*(N+1) +i*nu) << TF_MAX, TR_MAX, STEER_MAX;
        }

        //max velocity
        constraintMatrix((N+1)*nx+ 2*(N+1) + N*nu +i, i*nx+3) = 1;
        lower((N+1)*nx+ 2*(N+1) + N*nu +i) = 0;
        upper((N+1)*nx+ 2*(N+1) + N*nu +i) = SPEED_MAX;

        //max yaw_dot
        constraintMatrix((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i, i*nx+4) = 1;
        lower((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = -car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5)));
        ROS_INFO("MIN YAW DOT = %lf", -car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5))));
        upper((N+1)*nx + 2*(N+1)+N*nu+(N+1) +i) = car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5)));
        ROS_INFO("MAX YAW DOT = %lf", car.friction_coeff * 9.81 / (x_k_ref(3) * cos(x_k_ref(5))));

        // max slip angle
        constraintMatrix((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i, i*nx+5) = 1;
        lower((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = -atan(0.02 * car.friction_coeff * 9.81);
        upper((N+1)*nx + 2*(N+1)+N*nu+(N+1)*2 +i) = atan(0.02 * car.friction_coeff * 9.81);
        // ROS_INFO("MIN SLIP ANGLE = %lf", -atan(0.02 * car.friction_coeff * 9.81));
        // ROS_INFO("MAX SLIP ANGLE = %lf", atan(0.02 * car.friction_coeff * 9.81));

        // s_k >= 0
        constraintMatrix((N+1)*nx + 2*(N+1) + N*nu +3* (N+1) + i, (N+1)*nx+N*nu +i) = 1.0;
        lower((N+1)*nx + 2*(N+1) + N*nu  + 3 * (N+1) + i) = 0;
        upper((N+1)*nx + 2*(N+1) + N*nu  + 3 * (N+1) + i) = INFINITY;

        // max dertTf
        if (i < N && i > 0){
            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i, (N + 1)*nx + i * nu) = 1.0;
            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i, (N + 1)*nx + (i - 1) * nu) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i) = -DERT_TF_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + i) = DERT_TF_MAX * Ts;

            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i, (N + 1)*nx + i * nu + 1) = 1.0;
            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i, (N + 1)*nx + (i - 1) * nu + 1) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i) = -DERT_TR_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N + i) = DERT_TR_MAX * Ts;

            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i, (N + 1)*nx + i * nu + 2) = 1.0;
            constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i, (N + 1)*nx + (i - 1) * nu + 2) = -1.0;
            lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i) = -DERT_STEER_MAX * Ts;
            upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N * 2 + i) = DERT_STEER_MAX * Ts;

        } 
    }
    constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1), (N + 1)*nx) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1)) = TF_LAST - DERT_TF_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1)) = TF_LAST + DERT_TF_MAX * Ts;

    constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N, (N + 1)*nx + 1) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N) = TR_LAST - DERT_TR_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + N) = TR_LAST + DERT_TR_MAX * Ts;

    constraintMatrix((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N, (N + 1)*nx + 2) = 1.0;
    lower((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N) = STEER_LAST - DERT_STEER_MAX * Ts;
    upper((N+1)*nx + 2*(N+1) + N*nu + 4* (N+1) + 2 * N) = STEER_LAST + DERT_STEER_MAX * Ts;
    int numOfConstraintsSoFar = (N+1)*nx + 2*(N+1) + N*nu + 3* (N+1) + (N+1) + 3 * N;

    // lamda's >= 0
    for (int i=0; i<2*K_NEAR; i++){
        constraintMatrix(numOfConstraintsSoFar + i, (N+1)*nx+ N*nu + (N+1) + i) = 1.0;
        lower(numOfConstraintsSoFar + i) = 0;
        upper(numOfConstraintsSoFar + i) = 1.0;
    }
    numOfConstraintsSoFar += 2*K_NEAR;

    // terminal state constraints:  -s_t <= -x_N+1 + linear_combination(lambda's) <= s_t
    // 0 <= s_t -x_N+1 + linear_combination(lambda's) <= inf
    for (int i=0; i<2*K_NEAR; i++){
        for (int state_ind=0; state_ind<terminal_nx; state_ind++){
            constraintMatrix(numOfConstraintsSoFar + state_ind, (N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].x(state_ind);
        }
    }
    for (int state_ind=0; state_ind<terminal_nx; state_ind++){
        constraintMatrix(numOfConstraintsSoFar + state_ind, N*nx + state_ind) = -1;
        constraintMatrix(numOfConstraintsSoFar+state_ind, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + state_ind) = 1;
        lower(numOfConstraintsSoFar+state_ind) = 0.0;
        upper(numOfConstraintsSoFar+state_ind) = INFINITY;
    }
    numOfConstraintsSoFar += terminal_nx;

    //-inf <= -x_N+1 + linear_combination(lambda's) - s_t <= 0
    for (int i=0; i<2*K_NEAR; i++){
        for (int state_ind=0; state_ind<terminal_nx; state_ind++){
            constraintMatrix(numOfConstraintsSoFar + state_ind, (N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].x(state_ind);
        }
    }
    for (int state_ind=0; state_ind<terminal_nx; state_ind++){
        constraintMatrix(numOfConstraintsSoFar + state_ind, N*nx + state_ind) = -1;
        constraintMatrix(numOfConstraintsSoFar+state_ind, (N+1)*nx + N*nu + (N+1) + 2*K_NEAR + state_ind) = -1;
        lower(numOfConstraintsSoFar+state_ind) = -INFINITY;
        upper(numOfConstraintsSoFar+state_ind) = 0;
    }

    numOfConstraintsSoFar += terminal_nx;
   // cout<<"con dim: "<< (N+1)*nx+ 2*(N+1)*nx + N*nu + (N-1) + (N+1)*nx + (2*K_NEAR) + nx+1 <<endl;
    // sum of lamda's = 1;
    for (int i=0; i<2*K_NEAR; i++){
        constraintMatrix(numOfConstraintsSoFar, (N+1)*nx+ N*nu + (N+1) + i) = 1;
    }

    lower(numOfConstraintsSoFar) = 1.0;
    upper(numOfConstraintsSoFar) = 1.0;
    numOfConstraintsSoFar++;

    for(int i = 0; i < terminal_nx ; i++){
        constraintMatrix(numOfConstraintsSoFar + i, (N+1)*nx + N*nu + (N+1) + 2*K_NEAR + i) = 1.0;
        lower(numOfConstraintsSoFar + i) = 0;
        upper(numOfConstraintsSoFar + i) = INFINITY;
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
    for (int i=0; i<terminal_nx; i++){
        HessianMatrix((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + i, (N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + i) = q_s_terminal;
    }
//    for (int i=0; i<2*K_NEAR; i++){
//        gradient((N+1)*nx+ N*nu + (N+1) + i) = terminal_CSS[i].cost;
//    }

    // cout<<"gradient: "<<gradient<<endl;
    // cout<<"lower: "<<lower(517)<<endl;
    // cout<<"upper: "<<upper(517)<<endl;
    //x0 constraint
    lower.head(nx) = -x0;
    upper.head(nx) = -x0;

    // Eigen::MatrixXd H_t = HessianMatrix.transpose();
    // Eigen::MatrixXd sparse_I((N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);
    // sparse_I.setIdentity();
    // HessianMatrix = 0.5*(HessianMatrix + H_t) + 0.0000001*sparse_I;
    // ROS_INFO("HEREN");
    // cout << constraintMatrix<< endl;

    QProblem qpsolver(Hessian_row, ConstraintA_row);
    copy_Sparse_to_real_t(HHessianMatrix, HessianMatrix, Hessian_row, Hessian_col);
    copy_Sparse_to_real_t(ConstraintAMatrix, constraintMatrix, ConstraintA_row, ConstraintA_col);
    copy_Vector_to_real_t(lbA, lower, ConstraintA_row);
    copy_Vector_to_real_t(ubA, upper, ConstraintA_row);
    cout <<"lba = "<< *lbA << endl;
    
    int_t nWSR = 4000;
    qpsolver.init(HHessianMatrix, GMatrix, ConstraintAMatrix, 0, 0, lbA, ubA, nWSR);

    real_t xOpt[Hessian_row];
    real_t yOpt[Hessian_row+1];
	qpsolver.getPrimalSolution( xOpt );
	qpsolver.getDualSolution( yOpt );
    qpsolver.hotstart(GMatrix, 0, 0, lbA, ubA, nWSR);

    if(qpsolver.getPrimalSolution(xOpt)) {
        cout<< "fail to solve problem"<<endl;
        ros::shutdown();
        status_ = false;
        return;
    }
    visualize_mpc_solution(terminal_CSS, terminal_candidate);  // rviz可视化

//    cout<<"Solution lamd: "<<endl;
//     for (int index_lamd = 0; index_lamd < 2*K_NEAR; ++index_lamd){
//         cout<<QPSolution_((N+1)*nx + N* nu + N+1 + index_lamd)<< ",";
//     }
//     cout << endl;
//     cout<<"Solution s_k: "<<endl;
//     for (int index_sk = 0; index_sk < N + 1; ++index_sk){
//         cout<<QPSolution_((N+1)*nx + N* nu + index_sk)<< ",";
//     }
//     cout << endl;
//     cout<<"Solution s_t: "<<endl;
//     for (int index_st = 0; index_st < terminal_nx; ++index_st){
//         cout<<QPSolution_((N+1)*nx+ N*nu + (N+1) + 2*K_NEAR + index_st)<< ",";
//     }
    // cout << endl;
    // for (int index_v = 0; index_v < N + 1; ++index_v){
    //     cout<< "QPSOLUTION v = " << QPSolution_(index_v * nx + 3) << " QPSOLUTION r = " << QPSolution_(index_v * nx + 4) << " QPSOLUTION beta = " << QPSolution_(index_v * nx + 5)  << " QPSOLUTION ax = " << QPSolution_(index_v * nx + 6);
    //     if (index_v < N){
    //          cout << " QPSOLUTION Tf = " << QPSolution_((N+1) * nx + index_v * nu) <<  " QPSOLUTION delta = " << QPSolution_((N+1) * nx + index_v * nu + 2)<< endl;
    //     }
    // }
    copy_real_t_to_Vector(QPSolution_, xOpt, (N+1)*nx+ N*nu + (N+1)+ (2*K_NEAR) +terminal_nx);

    if (use_dyn_) ROS_INFO("using dynamics");
    else ROS_INFO("using kinematics");
}

void LMPC::copy_Sparse_to_real_t(real_t* target,
    Eigen::MatrixXd& source, int nRows,
    int nCols) {
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not
    // rows)
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count] = source(i, j);
            count++;
        }
    }
}

void LMPC::copy_Vector_to_real_t(real_t* target,
    Eigen::VectorXd& source, int length) {

    for (int i = 0; i < length; i++) {
        target[i] = source(i);
    }
}

void LMPC::copy_real_t_to_Vector(Eigen::VectorXd& target, real_t* source, int length) {

    for (int i = 0; i < length; i++) {
        target(i) = source[i];
    }
}

void LMPC::applyControl() {
    // cout<<"dert_Tf_cmd: "<<dert_Tf<<endl;
    // cout<<"dert_Tr_cmd: "<<dert_Tr<<endl;
    // cout << "dert_Steer_cmd: "<<dert_Steer<<endl;

    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = QPSolution_((N+1)*nx);
    cout<<"Tf_cmd: "<<QPSolution_((N+1)*nx)<<endl;
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
}

void LMPC::record_status(){
    std_msgs::String pred_state;
    stringstream ss_x, ss_y, ss_v, ss_yaw, ss_r, ss_slip_angle, ss_Tf, ss_Tr, ss_delta, ss_ax, ss_ay,  ss_control_Tf, ss_control_Tr, ss_control_steer, ss_cur_x, ss_cur_y, ss_cur_yaw, ss_cur_v, ss_cur_r, ss_cur_slip_angle, ss_cur_Tf, ss_cur_Tr, ss_cur_delta, ss_cur_ax, ss_cur_ay, ss_x_get, ss_y_get, ss_yaw_get, ss_error;
    string s_state, s_x, s_y, s_v, s_yaw, s_r, s_slip_angle, s_Tf, s_Tr, s_delta, s_ax, s_ay,  s_control_Tf, s_control_Tr, s_control_steer, s_cur_x, s_cur_y, s_cur_yaw, s_cur_v, s_cur_r, s_cur_slip_angle, s_cur_Tf, s_cur_Tr, s_cur_delta, s_cur_ax, s_cur_ay, s_x_get, s_y_get, s_yaw_get, s_error;
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
    ss_ax << QPSolution_(nx + 6);
    ss_ay << QPSolution_(nx + 7);
    ss_cur_x << QPSolution_(0);
    ss_cur_y << QPSolution_(1);
    ss_cur_yaw << QPSolution_(2);
    ss_cur_v << QPSolution_(3);
    ss_cur_r << QPSolution_(4);
    ss_cur_slip_angle << QPSolution_(5);
    ss_cur_Tf << Tf_;
    ss_cur_Tr << Tr_;
    ss_cur_delta << delta_;
    ss_cur_ax << QPSolution_(6);
    ss_cur_ay << QPSolution_(7);
    ss_x_get << car_pos_.x();
    ss_y_get <<  car_pos_.y();
    ss_yaw_get << yaw_;
    ss_error << error_num_;

    ss_x >> s_x;
    ss_y >> s_y;
    ss_yaw >> s_yaw;
    ss_v >> s_v;
    ss_r >> s_r;
    ss_slip_angle >> s_slip_angle;
    ss_Tf >> s_Tf;
    ss_Tr >> s_Tr;
    ss_delta >> s_delta;
    ss_ax >>s_ax;
    ss_ay >> s_ay;
    ss_cur_x >> s_cur_x;
    ss_cur_y >> s_cur_y;
    ss_cur_yaw >> s_cur_yaw;
    ss_cur_v >> s_cur_v;
    ss_cur_r >> s_cur_r;
    ss_cur_slip_angle >> s_cur_slip_angle;
    ss_cur_Tf >> s_cur_Tf;
    ss_cur_Tr >> s_cur_Tr;
    ss_cur_delta >> s_cur_delta;
    ss_cur_ax >> s_cur_ax;
    ss_cur_ay >> s_cur_ay;
    ss_x_get >> s_x_get;
    ss_y_get >> s_y_get;
    ss_yaw_get >> s_yaw_get;
    ss_error >> s_error;

    s_state = s_x+ comma+s_y+comma+ s_yaw + comma + s_v + comma + s_r + comma + s_slip_angle + comma+ s_Tf +comma+s_Tr + comma + s_delta +comma + s_ax + comma + s_ay + comma + s_cur_x + comma + s_cur_y + comma + s_cur_yaw + comma + s_cur_v + comma + s_cur_r + comma + s_cur_slip_angle + comma + s_cur_Tf +comma+ s_cur_Tr + comma + s_cur_delta + comma + s_cur_ax + comma + s_cur_ay + comma + s_x_get +comma + s_y_get + comma + s_yaw_get + comma + s_error;
    pred_state.data = s_state;
    pred_pub_.publish(pred_state);
}

void LMPC::visualize_mpc_solution(const vector<Sample>& convex_safe_set, const Eigen::Matrix<double,nx,1>& terminal_candidate) {
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
    Eigen::VectorXd costs = Eigen::VectorXd(convex_safe_set.size());
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
    ros::init(argc, argv, "LMPC_nonlinear_qp");
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
        lmpc.run();
        ts_calculate = clock();
        ROS_INFO("time_calculate = %f",(ts_calculate - ts_get_data)/CLOCKS_PER_SEC);
        rate.sleep();
        ts_end = clock();
        ROS_INFO("time = %f",(ts_end - ts_start)/CLOCKS_PER_SEC);
    }
    return 0;
}