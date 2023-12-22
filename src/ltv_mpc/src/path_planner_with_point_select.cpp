#include "controller_mpc/path_planner.h"
#include "controller_mpc/quintic_solver.hpp"
#include "iostream"
#include <Eigen/Dense>
#include <ctime>
#include <fstream>
#include <vector>
using namespace std;
using namespace Eigen;

PathPlanner::Planner::Planner() {

}

bool PathPlanner::Planner::loadPath(std::string filename, double scale) {//scale是放大缩小的倍数
    waypoints_.clear();
    vector<geometry_msgs::Point> waypoints_old;

    FILE *file = fopen(filename.c_str(), "rt");
    // std::cout << "here4.25" << std::endl;
    geometry_msgs::Point point;
    if(file != nullptr) {
        double x, y;
        while(fscanf(file, "%lf,%lf", &x, &y) != EOF) {
            point.x = scale * x;
            point.y = scale * y;
            point.z = 0;
            waypoints_old.push_back(point);
        }
        fclose(file);
        // std::cout << "here4.251" << std::endl;
        path_width =2;
        //进行点筛选，假设控制器实时性可以承受200个点
        //求所有点曲率
        vector<double> waypoint_r_collect;
        waypoint_r_collect.push_back(0);
        double r_waypoint, disp_t_waypoint_2, disp_t_waypoint;
        for(int i = 1;i < waypoints_old.size()-1;i++){
          if(abs(waypoints_old[i+1].x-waypoints_old[i].x)<1e-5||abs(waypoints_old[i].x-waypoints_old[i-1].x)<1e-5||abs(waypoints_old[i+1].x-waypoints_old[i-1].x)<1e-5){
            r_waypoint = 0;
          }
          else{
            disp_t_waypoint = (waypoints_old[i+1].y-waypoints_old[i].y)/(waypoints_old[i+1].x-waypoints_old[i].x);
            //cout << disp_t_start<<endl;
            disp_t_waypoint_2 = 2*((waypoints_old[i+1].y-waypoints_old[i].y)/(waypoints_old[i+1].x-waypoints_old[i].x)-(waypoints_old[i].y-waypoints_old[i-1].y)/(waypoints_old[i].x-waypoints_old[i-1].x))/(waypoints_old[i+1].x-waypoints_old[i-1].x);
            //cout << disp_t_start_2 <<endl;
            r_waypoint =  abs(disp_t_waypoint_2)/(pow(1+pow(disp_t_waypoint,2),(1.5)));
            double yaw_dert_waypoint = atan2(waypoints_old[i+1].y-waypoints_old[i].y,waypoints_old[i+1].x-waypoints_old[i].x)-atan2(waypoints_old[i].y-waypoints_old[i-1].y,waypoints_old[i].x-waypoints_old[i-1].x);
            if (yaw_dert_waypoint > M_PI){
              yaw_dert_waypoint = yaw_dert_waypoint-2*M_PI;
            }
            if (yaw_dert_waypoint < -M_PI){
              yaw_dert_waypoint = yaw_dert_waypoint +2*M_PI;
            }
            if(yaw_dert_waypoint<0){
              r_waypoint = -r_waypoint;
            }
          }
          waypoint_r_collect.push_back(r_waypoint);
        }
        waypoint_r_collect.push_back(0);
        vector<int> point_need_delete;
        /*
        //删除直线段的多余点
        double sum_cur = 0;
        for (int i = 1;i<waypoint_r_collect.size()-1;i++) {
          if(abs(waypoint_r_collect[i]) < 1e-3&&abs(waypoint_r_collect[i-1]) < 1e-3&&abs(waypoint_r_collect[i+1]) < 1e-3){
            point_need_delete.push_back(i+1);
          }
          else{
            sum_cur = sum_cur+ waypoint_r_collect[i]+1/path_width;
          }

        }
        */
        //cout << point_need_delete.size()<<endl;
        //判断当前点数是否大于200，如果大于200进行下一步筛选
        // std::cout << "here4.5" << std::endl;
        double sum_cur = 0;
        if (waypoints_old.size()-point_need_delete.size() >200){
          for (int i = 0;i<waypoint_r_collect.size();i++) {
            // sum_cur = sum_cur+ abs(waypoint_r_collect[i])+1/path_width;
            sum_cur = sum_cur+1/path_width;
          }
          //每50个点取一次曲率平均值
          int index_delect_point = 0;
          int num_need = 0;


          while (index_delect_point < waypoint_r_collect.size()){
            int num_point = 0;
            double sum_cur_sub = 0;
            while(true){
              if(num_point >= 50){
                break;
              }
              num_point+=1;
              // sum_cur_sub = sum_cur_sub + abs(waypoint_r_collect[index_delect_point])+1/path_width;
              sum_cur_sub = sum_cur_sub+1/path_width;
              index_delect_point+=1;
              if(index_delect_point >= waypoint_r_collect.size()){
                break;
              }
            }
            //cout << "here"<<endl;
            /*
            if(abs(waypoint_r_collect[index_delect_point])<1e-3){
              index_delect_point+=1;
              if (num_point < 1){
                continue;
              }
            }
            */
            num_need = ceil(200*(sum_cur_sub)/sum_cur);
            //cout << num_need;
            //cout << "index"<<index_delect_point<<endl;

            //cout << index_delect_point<<endl;
            if(num_need<num_point){
              int count = num_point/num_need -1;  //每隔多少点取一个
              int num_1 = num_point%num_need;  //前多少个是多一个间隔
              //cout <<num_point<<" "<< count <<" " << num_need<< " "<< num_1<<endl;
              int num_1_count = 0;  //记录多一个间隔的数量
              int num_count = 0;  //记录间隔几个了

              for(int i = 0;i<num_point;i++){

                if(num_1_count<num_1&&num_count < count+1){
                  point_need_delete.push_back(index_delect_point-i-1);
                  num_count ++;
                  //从当前waypoint_r_collect中倒叙找点
                }
                else if(num_1_count>=num_1&&num_count < count){
                  point_need_delete.push_back(index_delect_point-i-1);
                  num_count++;
                }
                else{
                  //cout << num_count<< "    !!"<<i<<endl;
                  num_count = 0;
                  num_1_count++;
                }
              }
            }
          //cout << "size =" << point_need_delete.size()<< "end ="<< point_need_delete[point_need_delete.size()-1]<<endl;
          }
          //cout << " !"<<index_delect_point<<endl;
        }
        for (int i = 0;i<waypoints_old.size();i++){
          if (std::find(point_need_delete.begin(), point_need_delete.end(), i) == point_need_delete.end()){
                      waypoints_.push_back(waypoints_old[i]);
          }
        }
        waypoints_.push_back(waypoints_old[waypoints_old.size()-1]);
        waypoints_.push_back(waypoints_old[0]);
        /*
        for (int i =0;i<97;i++){
          cout << point_need_delete[i]<<endl;
        }

        cout << "       "<<waypoints_[0].x<<" "<<waypoints_[0].y<<endl;
        */
      //  std::cout << "here5" << std::endl;

        computeSplineWithPoints();    //进行拟合

        return true;

    }

    return false;
}

void PathPlanner::Planner::computeSplineWithPoints() {
    int num_waypoints = int(waypoints_.size());
    //cout <<num_waypoints<<endl;
    if(num_waypoints <= 0) {
        return;
    }

    //create x and y line array
    vector<double> x_list(num_waypoints), y_list(num_waypoints), t_list(num_waypoints), s_list(num_waypoints);
    double t = 0;
    for(int i = 0; i < num_waypoints; i++) {
        x_list[i] = waypoints_[i].x;
        y_list[i] = waypoints_[i].y;
        if(i == 0){
          s_list[i] = 0.0;
        }else{
          s_list[i] = s_list[i - 1] + sqrt((y_list[i] - y_list[i- 1]) * (y_list[i] - y_list[i- 1]) + (x_list[i] - x_list[i- 1]) * (x_list[i] - x_list[i- 1]));
        }
        t_list[i] = t;
        t += 1;
    }

    spline_max_t_ = t_list[num_waypoints - 1];


    //interpolate parametric cubic spline
    spline_x_.set_points(t_list, x_list);
    spline_y_.set_points(t_list, y_list);
    spline_s_.set_points(t_list, s_list);
    // spline_x_ = CubicSpline::Natural(t_list, x_list);
    // spline_y_ = CubicSpline::Natural(t_list, y_list);
    goal_x_ = spline_x_(num_waypoints-1);
    goal_y_ = spline_y_(num_waypoints-1);
    total_s_ = spline_s_(num_waypoints-1);
    cout << goal_x_ <<" "<<goal_y_<<" "<<  total_s_<<endl;
    for(int i = 0; i < num_waypoints; i++){
      //cout <<  "noerror"<<endl;
      cx.push_back(spline_x_(i));
      cy.push_back(spline_y_(i));
      //cout << spline_x_(0) << " "<<spline_y_(0)<<endl;
      //cout << spline_x_(1) << " "<<spline_y_(1)<<endl;
      //cout << spline_x_(2) << " "<<spline_y_(2)<<endl;
      //cout << spline_x_(3) << " "<<spline_y_(3)<<endl;
    }
   ofstream outfile;
   ofstream outfile_xy;
   outfile.open("/home/sun234/racing_work/src/ltv_mpc/src/cur.txt");
   outfile_xy.open("/home/sun234/racing_work/src/ltv_mpc/src/xy.txt");
   /*
    std::string filename_2 = "/home/nvidia/vehicle_work/src/vehicle_work/src/cur_after.txt";
    FILE *file_2 = fopen(filename_2.c_str(), "rt");
    if(file_2 != nullptr) {
        double cur_n;
        while(fscanf(file_2, "%lf", &cur_n) != EOF) {
            ccur.push_back(cur_n);
        }
        fclose(file_2);
    }
    */
    cout << "spline_max_t_ = " << spline_max_t_ << endl;
    cout << "cvx size = " <<cvx.size() << endl;

    for(double i = 0.01; i < spline_max_t_; i=i+0.01){
        double kr_now;
        outfile_xy << spline_x_(i) << "," << spline_y_(i) << endl;
        if(abs(spline_x_(i+0.01)-spline_x_(i))<1e-5||abs(spline_x_(i)-spline_x_(i-0.01))<1e-5||abs(spline_x_(i+0.01)-spline_x_(i-0.01))<1e-5){
          kr_now = 0;
        }else{
            double disp_t_now = (spline_y_(i+0.01)-spline_y_(i))/(spline_x_(i+0.01)-spline_x_(i));
            double disp_t_now_2 = 2*((spline_y_(i+0.01)-spline_y_(i))/(spline_x_(i+0.01)-spline_x_(i))-(spline_y_(i)-spline_y_(i-0.01))/(spline_x_(i)-spline_x_(i-0.01)))/(spline_x_(i+0.01)-spline_x_(i-0.01));
            kr_now = abs(disp_t_now_2)/(pow(1+pow(disp_t_now,2),(1.5)));
            double yaw_dert_now = atan2(spline_y_(i+0.01)-spline_y_(i),spline_x_(i+0.01)-spline_x_(i))-atan2(spline_y_(i)-spline_y_(i-0.01),spline_x_(i)-spline_x_(i-0.01));
            if(yaw_dert_now > M_PI){
                yaw_dert_now -= 2 * M_PI;
            }
            if(yaw_dert_now < -M_PI){
                yaw_dert_now += 2 * M_PI;
            }
            if(yaw_dert_now < 0){
                kr_now = -kr_now;
            }
        }
        if (abs(kr_now) > 0.04){
          ccur.push_back(ccur[ccur.size() - 1]);
        }else{
          ccur.push_back(kr_now);
        }
        // ccur.push_back(kr_now);
        //cout <<"kr = "<< kr_now << " cvx = " << min(90/3.6, pow((0.7 * 9.8 / kr_now),0.5))<<endl;
    }
    ccur.push_back(ccur[ccur.size() - 1]);

    for(int i = 0; i < ccur.size(); i++){
        outfile << ccur[i] << endl;
        // outfile << min(36.0/3.6, pow((0.65 * 9.8 / ccur[i]),0.5)) << endl;
        cvx.push_back(min(36.0/3.6, pow((0.65 * 9.8 / ccur[i]),0.5)));
    }
    outfile.close();
    outfile_xy.close();

}

void PathPlanner::Planner::set_cvx(vector<double> carsim_s_list, vector<double> carsim_v_list){
  spline_s_v_.set_points(carsim_s_list, carsim_v_list);
  for(double i = 0; i <  ccur.size(); ++i){
    double s_curr_carsim = spline_s_(i * 0.01);
    cvx.at(i) = min(spline_s_v_(s_curr_carsim), pow((0.65 * 9.8 / ccur[i]),0.5));
  }
}

/*
    Assume that a, b >= 0
*/
// double PathPlanner::Planner::lengthOfSpline(double a, double b) {
//     const double gauss_quadrature_coeff[5][2] = {
//         {  0.0,                0.5688888888888889 },
//         { -0.5384693101056831, 0.47862867049936647 },
//         {  0.5384693101056831, 0.47862867049936647 },
//         { -0.906179845938664,  0.23692688505618908 },
//         {  0.906179845938664,  0.23692688505618908 }
//     };

//     double length = 0;
//     int max_segments = int(spline_x_.polynomials().size()) - 1;
//     for(int segment = int(a); segment <= std::min(int(b), max_segments); segment++) {
//         double start = std::max(a, double(segment));
//         double end   = std::min(b, double(segment + 1));

//         //get coefficients
//         const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
//         const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

//         //integrate this segment
//         double integral = 0;
//         for(int k = 0; k < 5; k++) {
//             double x = ((end - start) / 2) * gauss_quadrature_coeff[k][0] + ((start + end) / 2);

//             double A_x = coeff_x[1] + 2 * coeff_x[2] * x + 3 * coeff_x[3] * x * x;
//             double B_x = coeff_y[1] + 2 * coeff_y[2] * x + 3 * coeff_y[3] * x * x;
//             double F_x = std::sqrt( A_x * A_x + B_x * B_x );

//             integral += gauss_quadrature_coeff[k][1] * F_x;
//         }
//         integral *= (end - start) / 2;

//         length += integral;
//     }

//     return length;
// }

double PathPlanner::Planner::nearestPointOnSpline(double x, double y, double eps) {
    double min_dist_2 = 1.0 / 0.0;
    // cout << "here6.0.1"<< endl;
    double min_t = 0;
    cout <<spline_max_t_<<endl;
    for(double t = 0.01 ; t < spline_max_t_ ;t= t+0.01 ) {
        double dx = x - spline_x_(t);
        double dy = y - spline_y_(t);
        double dist_2 = sqrt(dx * dx + dy * dy);

        if(dist_2 < min_dist_2) {
            min_dist_2 = dist_2;
            min_t = t;
        }
    }
    // cout << "here6.0.2"<< endl;

    /*
    for(int segment = 0; segment < spline_max_t_; segment++) {
        //get 5th polynomial

            evaluated using sagemath

            a, b, c, d = var("a, b, c, d")
            e, f, g, h = var("e, f, g, h")
            x, y = var("x, y")

            f_x(t) = a + b * t + c * t^2 + d * t^3
            f_y(t) = e + f * t + g * t^2 + h * t^3

            df_x(t) = b + 2*c*t + 3*d*t^2
            df_y(t) = f + 2*g*t + 3*h*t^2

            -(
                (x - f_x(t)) * df_x(t) + (y - f_y(t)) * df_y(t)
            ).expand().collect(t)

        const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
        const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

        double a = coeff_x[0];
        double b = coeff_x[1];
        double c = coeff_x[2];
        double d = coeff_x[3];

        double e = coeff_y[0];
        double f = coeff_y[1];
        double g = coeff_y[2];
        double h = coeff_y[3];
        //cout << a <<" "<<b<<" "<< c<<" "<<d<<" "<<e<<" "<<f<<" "<<g<<" "<<h <<endl;


        QuinticSolver::QuinticPolynomial poly = {
            a*b + e*f - b*x - f*y, //0
            (b*b + 2*a*c + f*f + 2*e*g - 2*c*x - 2*g*y), //1
            3*(b*c + a*d + f*g + e*h - d*x - h*y), //2
            2*(c*c + 2*b*d + g*g + 2*f*h), //3
            5*(c*d + g*h), //4
            3*(d*d + h*h) //5
        };

        //solve 5th polynomial
        std::vector<double> sol_t;
        QuinticSolver::solveRealRootsSpecialForCubicSplineDerivative(poly, double(segment), double(segment + 1), &sol_t, eps);   //对五次多项式进行求解

        */
        //cout << "double" << endl;
        //get nearest
        /*
        for(auto it = sol_t.begin(); it != sol_t.end(); it++) {
            double t  = *it;
            cout <<"t in " <<endl;
            double dx = x - spline_x_(t);
            double dy = y - spline_y_(t);
            double dist_2 = dx * dx + dy * dy;

            if(dist_2 < min_dist_2) {
                min_dist_2 = dist_2;
                min_t = t;
            }
        }





    }
*/
    //cout << "x"<<x<<"y"<<y<<endl;
    double dx_zero =  x - waypoints_[0].x;
    double dy_zero = y - waypoints_[0].y;
    //cout << "dx_zero"<<dx_zero<<"dy_zero"<<dy_zero<<endl;
    double dist_zero = sqrt(pow(dx_zero,2) + pow(dy_zero,2));
    //cout << "dist_zero"<< dist_zero<<" " <<min_dist_2<<endl;
    //cout << min_t<<endl;

    if (dist_zero < min_dist_2)
    {
      min_dist_2 = dist_zero;
      //cout <<"imsdddsdsssds"<<endl;
      min_t = 0.0001;
    }
    // cout << "here6.0.3"<< endl;



    //cout << "acd" <<min_t << endl;
    error_track = min_dist_2;
    if ((y - spline_y_(min_t)) * std::cos(std::atan2(spline_y_.eval_d(min_t), spline_x_.eval_d(min_t))) - (x - spline_x_(min_t))  * std::sin(std::atan2(spline_y_.eval_d(min_t), spline_x_.eval_d(min_t))) < 0)
    {
      error_track = -error_track;
    }
//cout<< "min_t"<<min_t<<endl;
//cout<< "min_d"<<min_dist_2<<endl;
    return min_t;
}

double PathPlanner::Planner::pointWithFixedDistance(double t, double d, double eps) {
    if(d < eps) return t;



    //bi-section
    double l = t;
    double r = spline_max_t_;
    if(spline_s_(r) - spline_s_(l) < d){
      d = d - (spline_s_(r) - spline_s_(l));
      t = 0;
      l = t;
    }

    while((r - l) > eps) {  //二分法求预瞄点
        double m = (l + r) / 2;

        if(spline_s_(m) - spline_s_(t) <= d) l = m;
        else r = m;
    }

    return (l + r) / 2;
}

// double PathPlanner::Planner::maxCurvature(double l, double r, double numerical_eps) {
//     double max_curvature = 0;

//     int max_segments = int(spline_x_.polynomials().size()) - 1;
//     for(int segment = int(l); segment <= std::min(int(r), max_segments); segment++) {
//         double start = std::max(l, double(segment));
//         double end   = std::min(r, double(segment + 1));

//         const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
//         const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

//         //quadratic equation of curvature = a * x^2 + b * x + c
//         double a = 36 * (coeff_x[3] * coeff_x[3] + coeff_y[3] * coeff_y[3]);
//         double b = 24 * (coeff_x[2] * coeff_x[3] + coeff_y[2] * coeff_y[3]);
//         double c = 4  * (coeff_x[2] * coeff_x[2] + coeff_y[2] * coeff_y[2]);

//         if(std::fabs(b) > numerical_eps) {
//             double x = -a / b;
//             if((x >= start) && (x <= end)) max_curvature = std::max(max_curvature, c + x * (b + x * a));
//         }

//         max_curvature = std::max(max_curvature, c + start * (b + start * a));
//         max_curvature = std::max(max_curvature, c +   end * (b + end   * a));
//     }

//     return max_curvature;
// }

void PathPlanner::Planner::getPath(const PathPlanner::PoseStamped &cur_pose, double dt, double v_ref, double v_min, double k, double max_brake_accel, int n, std::vector<PathPlanner::PoseStamped> *path) {
    //initial pose
    path->clear();

    //find nearest point on trajectory
    double t_start = nearestPointOnSpline(cur_pose.x, cur_pose.y);   //tstart为最近点位置
    // cout << "here6.1"<< endl;
    vector<double> t_clear;
    state_t.swap(t_clear);
    state_t.push_back(t_start);
    path->push_back(PathPlanner::PoseStamped({
        spline_x_(t_start), spline_y_(t_start), std::atan2(spline_y_.eval_d(t_start), spline_x_.eval_d(t_start)),
        cvx[t_start / 0.01],
        0.0
    }));
    // cout << "here6.2"<< endl;
    //cout <<cvx.size() << " t " << t_start / 0.01 << " cvx_now =  " <<cvx[t_start / 0.01]<<endl;



    //calculate max curvature radius
    /*double brake_time = std::max(v_min, cur_pose.v) / max_brake_accel;
    double horizon_length = cur_pose.v * brake_time - 0.5 * max_brake_accel * brake_time * brake_time;
    double max_curvature = maxCurvature(t_start, pointWithFixedDistance(t_start, horizon_length));

    //calculate speed
    double eps = 1e-8;
    double v   = v_ref;
    if(max_curvature > eps) {
        //max velocity at certain radius
        double max_v = k * (1 / max_curvature);

        v = std::min(v, std::max(v_min, max_v));
    }

    printf("h: %lf, v: %lf\n", horizon_length, v);*/

    double v   = v_ref;

    double time    = dt;

    vector<double> cur_clear;
    curvature_cur.swap(cur_clear);

    //计算当前点曲率
    /*
    const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[t_start].coefficients();
    const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[t_start].coefficients();


    double a = coeff_x[0];
    double b = coeff_x[1];
    double c = coeff_x[2];

    double e = coeff_y[0];
    double f = coeff_y[1];
    double g = coeff_y[2];

    double x_dot = 3 * a * t_start * t_start +2 * b * t_start + c;
    double y_dot = 3 * e * t_start * t_start +2 * f * t_start + g;
    double x_dot_2 = 6 * a * t_start + 2 * b;
    double y_dot_2 = 6 * e * t_start + 2 * f;
    double dy_dx =y_dot/x_dot;
    double dy_dx_2 =y_dot_2 / x_dot_2;
    double kr_tstart = dy_dx_2/(pow((1+pow(dy_dx,2)),3/2));
    */
    /*
    double kr_tstart;
    if(abs(spline_x_(t_start+0.01)-spline_x_(t_start))<1e-5||abs(spline_x_(t_start)-spline_x_(t_start-0.01))<1e-5||abs(spline_x_(t_start+0.01)-spline_x_(t_start-0.01))<1e-5){
      kr_tstart = 0;
    }
    else{
      double disp_t_start = (spline_y_(t_start+0.01)-spline_y_(t_start))/(spline_x_(t_start+0.01)-spline_x_(t_start));
      //cout << disp_t_start<<endl;
      double disp_t_start_2 = 2*((spline_y_(t_start+0.01)-spline_y_(t_start))/(spline_x_(t_start+0.01)-spline_x_(t_start))-(spline_y_(t_start)-spline_y_(t_start-0.01))/(spline_x_(t_start)-spline_x_(t_start-0.01)))/(spline_x_(t_start+0.01)-spline_x_(t_start-0.01));
      //cout << disp_t_start_2 <<endl;
      kr_tstart =  abs(disp_t_start_2)/(pow(1+pow(disp_t_start,2),(1.5)));
      double yaw_dert = atan2(spline_y_(t_start+0.01)-spline_y_(t_start),spline_x_(t_start+0.01)-spline_x_(t_start))-atan2(spline_y_(t_start)-spline_y_(t_start-0.01),spline_x_(t_start)-spline_x_(t_start-0.01));
      if (yaw_dert > M_PI){
        yaw_dert = yaw_dert-2*M_PI;
      }
      if (yaw_dert < -M_PI){
        yaw_dert = yaw_dert +2*M_PI;
      }
      if(yaw_dert<0){
        kr_tstart = -kr_tstart;
      }
    }
    */

    //cout <<" kr_tstart=" <<kr_tstart<<endl;

    curvature_cur.push_back(ccur[t_start / 0.01]);
    double l_next = 0.0;

    for(int i = 0; i < n-1; i++) {
        l_next += cvx[state_t[state_t.size()- 1] / 0.01] * dt;
      //cout << i<<endl;
        double t = pointWithFixedDistance(t_start, l_next);  //求得预瞄点，输入的是当前点时间，后按照v*time找到预瞄点时间
        //cout << "t = "<<t << spline_x_(t)<< ' ' << spline_y_(t)<<endl;
        path->push_back(PathPlanner::PoseStamped({
            spline_x_(t), spline_y_(t), std::atan2(spline_y_.eval_d(t), spline_x_.eval_d(t)),
            cvx[t / 0.01],
            time
        }));
        // cout << "here6.3"<< endl;
        state_t.push_back(t);

        /*
        const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[t].coefficients();
        const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[t].coefficients();

        double a = coeff_x[0];
        double b = coeff_x[1];
        double c = coeff_x[2];

        double e = coeff_y[0];
        double f = coeff_y[1];
        double g = coeff_y[2];

        double x_dot = 3 * a * t * t +2 * b * t + c;
        double y_dot = 3 * e * t * t +2 * f * t + g;
        double x_dot_2 = 6 * a * t + 2 * b;
        double y_dot_2 = 6 * e * t + 2 * f;
        double dy_dx =y_dot/x_dot;
        double dy_dx_2 =y_dot_2 / x_dot_2;
        double kr = dy_dx_2/(pow((1+pow(dy_dx,2)),3/2));
        */
        double disp_t = 0;
        double disp_t_2 =0;
        double kr;
        /*
        if (t+0.01 < spline_max_t_){
          if(abs(spline_x_(t+0.01)-spline_x_(t))<1e-5||abs(spline_x_(t)-spline_x_(t-0.01))<1e-5||abs(spline_x_(t+0.01)-spline_x_(t-0.01))<1e-5){
            kr = 0;
          }
          else {
            disp_t = (spline_y_(t+0.01)-spline_y_(t))/(spline_x_(t+0.01)-spline_x_(t));

           //cout <<disp_t<<endl;
            disp_t_2 = 2*((spline_y_(t+0.01)-spline_y_(t))/(spline_x_(t+0.01)-spline_x_(t))-(spline_y_(t)-spline_y_(t-0.01))/(spline_x_(t)-spline_x_(t-0.01)))/(spline_x_(t+0.01)-spline_x_(t-0.01));
            kr =  abs(disp_t_2)/pow((1.0+disp_t*disp_t),1.5);
            double yaw_dert_kr_1 = atan2(spline_y_(t+0.01)-spline_y_(t),spline_x_(t+0.01)-spline_x_(t))-atan2(spline_y_(t)-spline_y_(t-0.01),spline_x_(t)-spline_x_(t-0.01));
            if (yaw_dert_kr_1 > M_PI){
              yaw_dert_kr_1 = yaw_dert_kr_1-2*M_PI;
            }
            if (yaw_dert_kr_1 < -M_PI){
              yaw_dert_kr_1 = yaw_dert_kr_1 +2*M_PI;
            }
            if(yaw_dert_kr_1<0){
              kr = -kr;
            }
          }

        }
        else {
          if (abs(spline_x_(t+0.01)-spline_x_(t))<1e-5||abs(spline_x_(t)-spline_x_(t-0.01))<1e-5){
            kr = 0;
          }
          else {
            disp_t = (spline_y_(spline_max_t_)-spline_y_(t))/(spline_x_(spline_max_t_)-spline_x_(t));

           //cout <<disp_t<<endl;
            disp_t_2 = 2*((spline_y_(spline_max_t_)-spline_y_(t))/(spline_x_(spline_max_t_)-spline_x_(t))-(spline_y_(t)-spline_y_(t-0.01))/(spline_x_(t)-spline_x_(t-0.01)))/(spline_x_(spline_max_t_)-spline_x_(t-0.01));
            kr =  abs(disp_t_2)/pow((1.0+disp_t*disp_t),1.5);
            double yaw_dert_kr_2 = atan2(spline_y_(spline_max_t_)-spline_y_(t),spline_x_(spline_max_t_)-spline_x_(t))-atan2(spline_y_(t)-spline_y_(t-0.01),spline_x_(t)-spline_x_(t-0.01));
            if (yaw_dert_kr_2 > M_PI){
              yaw_dert_kr_2 = yaw_dert_kr_2-2*M_PI;
            }
            if (yaw_dert_kr_2 < -M_PI){
              yaw_dert_kr_2 = yaw_dert_kr_2 +2*M_PI;
            }
            if(yaw_dert_kr_2<0){
              kr = -kr;
            }
          }

        }
        */
        //cout <<disp_t_2<<endl;

        //cout << i<<endl;
        //cout <<"kr"<<kr<<endl;
        curvature_cur.push_back(ccur[t / 0.01]);   //curvature存储着从当前点到预瞄距离的所有点曲率

        time = time+dt;

    }

}

double PathPlanner::Planner::randomFloat() {
    static std::random_device rd;
    static std::mt19937 e(rd());
    static std::uniform_real_distribution<> dist(0, 1);

    return dist(e);
}

// void PathPlanner::Planner::runTests() {
//     fprintf(stderr, "PathPlanner Tests begin.\n");

//     clock_t t_start, t_end;
//     double t_total;
//     int n_compute;

//     //spline length
//     t_start = clock();
//     n_compute = 0;

//     for(int i = 0; i < 100; i++) {
//         double start_point = randomFloat() * spline_max_t_;

//         double dt = spline_max_t_ / 10000;
//         double last_len = -1;

//         for(double t = start_point; t < spline_max_t_; t += dt) {
//             double len = lengthOfSpline(0, t);
//             n_compute++;

//             assert(len > last_len);

//             last_len = len;
//         }
//     }

//     t_end = clock();
//     t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

//     fprintf(stderr, "[OK] Spline length, %lf msec/call.\n", t_total);

//     //point on spline
//     t_start = clock();
//     n_compute = 0;

//     double max_error = 0;
//     for(int i = 0; i < 10000; i++) {
//         double t = randomFloat() * spline_max_t_;
//         double x = spline_x_(t);
//         double y = spline_y_(t);

//         double t_estimated = nearestPointOnSpline(x, y);
//         double x_estimated = spline_x_(t_estimated);
//         double y_estimated = spline_y_(t_estimated);

//         double error = std::sqrt((x - x_estimated) * (x - x_estimated) + (y - y_estimated) * (y - y_estimated));
//         max_error = std::max(max_error, error);

//         n_compute++;
//     }

//     t_end = clock();
//     t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

//     fprintf(stderr, "[OK] Spline point, max error = %lf, %lf msec/call.\n", max_error, t_total);

//     //point with fixed distance
//     t_start = clock();
//     n_compute = 0;

//     max_error = 0;
//     for(int i = 0; i < 10000; i++) {
//         double t = randomFloat() * spline_max_t_;
//         double d = randomFloat() * lengthOfSpline(t, spline_max_t_);

//         double t_next = pointWithFixedDistance(t, d);
//         assert(t_next >= t);

//         double error = std::fabs(lengthOfSpline(t, t_next) - d);

//         max_error = std::max(max_error, error);

//         n_compute++;
//     }

//     t_end = clock();
//     t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

//     fprintf(stderr, "[OK] Point with distance, max error = %lf, %lf msec/call.\n", max_error, t_total);
// }
