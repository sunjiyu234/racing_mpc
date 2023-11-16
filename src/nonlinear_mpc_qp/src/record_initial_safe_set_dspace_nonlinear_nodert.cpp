//
// Created by yuwei on 4/2/20.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nonlinear_mpc_qp/track.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Sparse>

using namespace std;

class Record_SS{
public:
    Record_SS(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;
    double s_prev_;
    int lap_;
    int time_;
    ofstream data_file_;
    double ros_time_prev_;
    nav_msgs::Odometry odom_;
    sensor_msgs::Imu state_;
    double acc_cmd_;


    Track* track_;
    void cmd_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &cmd_msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void accel_cmd_callback(const std_msgs::Float32ConstPtr & accel_cmd);
    void state_callback(const sensor_msgs::Imu::ConstPtr &state_msg);
    Eigen::Vector3d global_to_track(double x, double y, double yaw, double s);
};

Record_SS::Record_SS(ros::NodeHandle &nh) : nh_(nh) {
    cmd_sub_ = nh_.subscribe("nav", 1, &Record_SS::cmd_callback, this);   
    odom_sub_ = nh_.subscribe("odom", 1, &Record_SS::odom_callback, this);
    state_sub_ = nh_.subscribe("state_odom", 1, &Record_SS::state_callback, this);

    string wp_file;
    double space;
    nh.getParam("waypoint_file", wp_file);
    nh.getParam("WAYPOINT_SPACE", space);

    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));
    if (map_ptr == nullptr){
        ROS_INFO("No map received");
        track_ = new Track(wp_file, true);    
    }
    else{
        ROS_INFO("Map received");
        nav_msgs::OccupancyGrid map = *map_ptr;
        track_ = new Track(wp_file, map, true);
    }

    nav_msgs::Odometry odom_msg;
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", ros::Duration(5));
    if (odom_ptr == nullptr){cout<< "fail to receive odom message!"<<endl;}
    else{
        odom_msg = *odom_ptr;
    }
    float x = odom_msg.pose.pose.position.x;
    float y = odom_msg.pose.pose.position.y;
    s_prev_ = track_->findTheta(x,y,0,true);   // min_ind * space

    ros_time_prev_ = ros::Time::now().toSec();
    time_=0; lap_=0;

    data_file_.open( "/home/sun234/racing_work/src/ltv_mpc/src/initial_safe_set_dspace_new_10ms_nodert.csv");
}

void Record_SS::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
    odom_ = *odom_msg;
}

void Record_SS::accel_cmd_callback(const std_msgs::Float32ConstPtr & accel_cmd){
    acc_cmd_ = accel_cmd->data;
}

void Record_SS::state_callback(const sensor_msgs::Imu::ConstPtr &state_msg){
    state_ = *state_msg;
}

void Record_SS::cmd_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &cmd_msg) {

    if (cmd_msg != nullptr){
        cout<<"recording: "<<time_<<endl;
        double Tf_cmd = cmd_msg->drive.speed;
        double Tr_cmd = cmd_msg->drive.acceleration;
        double steer_cmd = cmd_msg->drive.steering_angle * M_PI / 180.0;

        double yaw = tf::getYaw(odom_.pose.pose.orientation);
        float x = odom_.pose.pose.position.x;
        float y = odom_.pose.pose.position.y;
        double s_curr = track_->findTheta(x,y,0,true);
        double vel = sqrt(pow(odom_.twist.twist.linear.x , 2) + pow(odom_.twist.twist.linear.y, 2));
        double yawdot = odom_.twist.twist.angular.z;
        double slip_angle = atan2(odom_.twist.twist.linear.y, odom_.twist.twist.linear.x);
        double Tf = state_.angular_velocity.x;
        double Tr = state_.angular_velocity.y;
        double delta = state_.angular_velocity.z;
        double ax = state_.linear_acceleration.x;
        double ay = state_.linear_acceleration.y;

        // check if is a new lap;
        if (s_curr - s_prev_ < -track_->length/2){
//            cout<<"s_curr: "<<s_curr<<endl;
//            cout<<"s_prev: "<<s_prev_<<endl;
            time_ = 0;
            lap_++;
            if (lap_>1){ //initial two laps completed
                data_file_.close();
                delete(track_);
                ros::shutdown();
            }
        }
        data_file_ << time_ <<","<<x<< ","<<y<<","<< s_curr << "," << yaw << "," << vel<<","<<yawdot<<","<<slip_angle<<","<< Tf << "," << Tr << "," << delta << "," << ax << "," << ay << "," <<Tf_cmd<<","<<Tr_cmd<<","<<steer_cmd<<endl;
        time_++;

        s_prev_ = s_curr;
        double ros_time_curr = ros::Time::now().toSec();
        cout << "dt: "<< ros_time_curr - ros_time_prev_ <<endl;
        ros_time_prev_ = ros_time_curr;
    }
}

Eigen::Vector3d Record_SS::global_to_track(double x, double y, double yaw, double s){
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

int main(int argc, char **argv){
    ros::init(argc, argv, "record_init_ss_nonlinear_qp");
    ros::NodeHandle nh;
    Record_SS rec_ss(nh);
    ros::spin();
    return 0;
}