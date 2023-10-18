#ifndef NMPC_H
#define NMPC_H

#include "path_planner.h"
#include "osqp.h"
#include <vector>

namespace NMPC {
    struct HardConstraint{
        double max_steer;

        double max_steer_rate;
        double max_ey;
        double max_eyaw;
    };

    struct CostFunctionWeights{
      double w_r;
      double w_v_y;
      double w_e_yaw;
      double w_e_d;

      double w_steer_angle;
      double w_slack_eyaw;
      double w_slack_ey;
    };

    struct State{
      double yaw_rate;
      double v_y;
      double e_yaw;
      double e_d;

      double steer_angle;
    };

    struct Parameters{
      int pred_horizon;
      int control_horizon;

      double dt;
    };

    struct Model {
      double l_f;
      double l_r;
      double m;
      double D_f;
      double D_r;
      double B_f;
      double B_r;
      double C_f;
      double C_r;
      double E_f;
      double E_r;
      double l;
      double I_z;
      double h;
      double kr;
      double g;
      double vx;
    };

    struct ControlOutput{
      int error_code;
      double steer;
    };

    template<typename T>
    struct SparseMatrixElement {
        int r, c;
        T v;

        inline bool operator<(const SparseMatrixElement &rhs) {
            return (c == rhs.c) ? (r < rhs.r) : (c < rhs.c);
        }
    };

    template<typename T>
    class SparseMatrix {
    private:
        int m_, n_;

        std::vector< SparseMatrixElement<T> > elements_;

        std::vector<T> osqp_csc_data_;
        std::vector<c_int> osqp_csc_row_idx_;
        std::vector<c_int> osqp_csc_col_start_;
        csc *osqp_csc_instance = nullptr;

        void freeOSQPCSCInstance();


    public:
        SparseMatrix();
        ~SparseMatrix();

        void initialize(int m, int n);
        void addElement(int r, int c, T v);
        void ShowFun(int m, int n);
        csc *toOSQPCSC();
    };

    template<typename T>
    class QPProblem {
    private:
        OSQPWorkspace *osqp_workspace_ = nullptr;
        OSQPSettings  *osqp_settings_= nullptr;
        OSQPData      *osqp_data_ = nullptr;

    public:
        //number of variables and constraints
        int n_, m_;

        //constraints
        SparseMatrix<T> A_;   //稀疏矩阵A
        std::vector<T> l_, u_;

        //cost function
        SparseMatrix<T> P_;
        std::vector<T> q_;

        ~QPProblem();
        void initialize(int n, int m);
        OSQPSolution* solve(int *error_code);
    };

    class Controller {
    private:
      QPProblem<c_float> qp_;

    public:
      /* controller parameters */
      HardConstraint constraint_;
      CostFunctionWeights weights_;
      Parameters parameters_;
      Model model_;

      Controller();

      void initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights);
      void update(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, const std::vector<double> &curvature_cur ,ControlOutput *out, std::vector<State> *pred_out);
    };

    class IterativeController : public Controller {
    public:
        void simulate(const State &state, const std::vector<double> &curvature, State *next);
        void update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold, const std::vector<double> &curvature_cur, ControlOutput *out, std::vector<State> *pred_out);
    };

}

#endif // NMPC_H
