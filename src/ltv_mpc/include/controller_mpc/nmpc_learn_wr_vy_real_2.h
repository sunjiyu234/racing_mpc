#ifndef NMPC_LEARN_H
#define NMPC_LEARN_H

#ifndef NMPC_H
#define NMPC_H

#include "path_planner.h"
#include "osqp.h"
#include <vector>

namespace NMPC_learn {
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
    struct data_state{
      double r;
      double vy;
      double eyaw;
      double ey;
      double cur;
      double steer;
      double min_t;
    };
    struct pred_state{
      double r;
      double vy;
      double e_yaw;
      double e_y;
      double cur;
      double steer;
      double min_t;
    };
    struct pred_state_005{
      double r;
      double vy;
      double e_yaw;
      double e_y;
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
    struct OptimizeOutput{
      int error_code;
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


    public:
      /* controller parameters */
      QPProblem<c_float> qp_;
      QPProblem<c_float> qp_optimize;
      HardConstraint constraint_;
      CostFunctionWeights weights_;
      Parameters parameters_;
      Model model_;




      vector<int> index_x;
      vector<double> x_norm;

      Controller();

      void initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights);
      void update(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, const std::vector<double> &curvature_cur ,ControlOutput *out, std::vector<State> &pred_out, std::vector<double> &control_out);
      void optimize(vector<int> index_x,vector<double> x_norm, double sum_x_norm, vector<data_state> nmpc_data_collect, vector<vector<pred_state_005>> pred_data_collect_next, OptimizeOutput *optimize_error ,vector<vector<double>> &optimize_abc);
      void optimize_true(vector<pred_state> min_pred_state,int i,OptimizeOutput *optimize_error, vector<vector<double>> &optimize_abc, const std::vector<double> &curvature_cur, const State &linearize_point );
      void update_learn(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, const std::vector<double> &curvature_cur ,ControlOutput *out, std::vector<State> &pred_out, std::vector<double> &control_out, vector<vector<double>> error_wr_,vector<vector<double>> error_vy_,vector<vector<double>> &dert_wr_,vector<vector<double>> &dert_vy_, vector<vector<double>> &dert_e_yaw_,vector<vector<double>>&dert_e_y_, vector<vector<double>> &dert_wr_correct,vector<vector<double>> &dert_vy_correct, vector<vector<double>> &dert_e_yaw_correct,vector<vector<double>>&dert_e_y_correct);
    };

    class IterativeController : public Controller {
    public:
      double error_wr_correct;
      double error_vy_correct;
      double error_eyaw_correct;
      double error_ey_correct;
        void simulate(const State &state, const std::vector<double> &curvature, State *next);
        void update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold, const std::vector<double> &curvature_cur, ControlOutput *out, std::vector<State> &pred_out, std::vector<data_state> &nmpc_data_collect, std::vector<vector<pred_state>> &pred_data_collect, vector<double> state_t);
        void update_learn(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold, const std::vector<double> &curvature_cur, ControlOutput *out, std::vector<State> &pred_out, std::vector<data_state> &nmpc_data_collect, std::vector<vector<pred_state>> &pred_data_collect, std::vector<vector<pred_state_005>> &pred_data_collect_next, std::vector<vector<pred_state_005>> &pred_data_collect_next_correct,vector<double> state_t);
    };

}

#endif // NMPC_H


#endif // NMPC_LEARN_H
