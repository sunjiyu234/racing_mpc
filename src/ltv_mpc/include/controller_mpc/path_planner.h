#ifndef __CAR_MODEL_CONTROLLER_MPC_PATH_PLANNER_H__
#define __CAR_MODEL_CONTROLLER_MPC_PATH_PLANNER_H__

#include <string>
#include <cmath>
#include <algorithm>

#include "ros/ros.h"
#include <ecl/geometry.hpp>
#include <ecl/geometry/polynomial.hpp>
#include <nav_msgs/Path.h>
#include <vector>
#include <Eigen/Dense>
#include <controller_mpc/spline.h>

using ecl::CubicSpline;
using namespace std;
using namespace Eigen;

namespace PathPlanner {
    struct PoseStamped {
        double x, y, yaw;
        double v;
        double t;
    };

    class Planner {
    private:
        std::vector<geometry_msgs::Point> waypoints_;
        tk::spline spline_x_;
        tk::spline spline_y_;
        tk::spline spline_s_;
        double spline_max_t_;


        void computeSplineWithPoints();
        double nearestPointOnSpline(double x, double y, double eps = 1e-8);
        double lengthOfSpline(double a, double b);
        double pointWithFixedDistance(double t, double d, double eps = 1e-8);
        double maxCurvature(double l, double r, double numerical_eps = 1e-8);

        double randomFloat();
    public:
        Planner();
        double goal_x_;
        double goal_y_;
        double total_s_;
        double min_t;
        double error_track;
        double path_width;
        vector<double> curvature_cur;
        vector<double> state_t;
        vector<double> cx;
        vector<double> cy;
        vector<double> cvx;
        vector<double> ccur;
        bool loadPath(std::string filename, double scale = 1);
        void getPath(const PathPlanner::PoseStamped &cur_pose, double dt, double v_ref, double v_min, double k, double max_brake_accel, int n, std::vector<PathPlanner::PoseStamped> *path);
        // void runTests();
    };
}

#endif
