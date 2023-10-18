#include "controller_mpc/nmpc_dspace.h"
#include "float.h"
#include <math.h>
#include "controller_mpc/path_planner.h"
#include <vector>
#include <Eigen/LU>
#include <Eigen/Core>



template
class NMPC::SparseMatrix<c_float>;

template
class NMPC::QPProblem<c_float>;

template<typename T>
NMPC::SparseMatrix<T>::SparseMatrix() {
    m_ = n_ = 0;
}

template<typename T>
NMPC::SparseMatrix<T>::~SparseMatrix() {
    elements_.clear();
    freeOSQPCSCInstance();
}



template<typename T>
void NMPC::SparseMatrix<T>::freeOSQPCSCInstance() {
    osqp_csc_data_.clear();
    osqp_csc_row_idx_.clear();
    osqp_csc_col_start_.clear();

    if(osqp_csc_instance != nullptr) {
        c_free(osqp_csc_instance);
        osqp_csc_instance = nullptr;
    }
}

template<typename T>
void NMPC::SparseMatrix<T>::initialize(int m, int n) {
    m_ = m;
    n_ = n;
    elements_.clear();
}

template<typename T>
void NMPC::SparseMatrix<T>::addElement(int r, int c, T v) {
    elements_.push_back({r, c, v});
}


template<typename T>
csc* NMPC::SparseMatrix<T>::toOSQPCSC() {
    freeOSQPCSCInstance();

    sort(elements_.begin(), elements_.end());

    int idx = 0;
    int n_elem = elements_.size();
    //cout << n_elem <<endl;

    osqp_csc_col_start_.push_back(0);
    for(int c = 0; c < n_; c++) {
        while((idx < n_elem) && elements_[idx].c == c) {
            osqp_csc_data_.push_back(elements_[idx].v);
            osqp_csc_row_idx_.push_back(elements_[idx].r);
            //cout << elements_[idx].v<<endl;
            idx++;
        }

        osqp_csc_col_start_.push_back(osqp_csc_data_.size());
    }

    osqp_csc_instance = csc_matrix(m_, n_, osqp_csc_data_.size(), osqp_csc_data_.data(), osqp_csc_row_idx_.data(), osqp_csc_col_start_.data());
    return osqp_csc_instance;
}

template<typename T>
NMPC::QPProblem<T>::~QPProblem() {
    if(osqp_workspace_ != nullptr) {
        osqp_workspace_->data->P = nullptr;
        osqp_workspace_->data->q = nullptr;

        osqp_workspace_->data->A = nullptr;
        osqp_workspace_->data->l = nullptr;
        osqp_workspace_->data->u = nullptr;

        //cleanup workspace
        osqp_cleanup(osqp_workspace_);
    }
}

template<typename T>
void NMPC::QPProblem<T>::initialize(int n, int m) {
    n_ = n;
    m_ = m;

    A_.initialize(m_, n_);
    l_.resize(m_);
    u_.resize(m_);

    std::fill(l_.begin(), l_.end(), 0);    //把l和u的所有值都赋为0
    std::fill(u_.begin(), u_.end(), 0);

    P_.initialize(n_, n_);
    q_.resize(n_);

    std::fill(q_.begin(), q_.end(), 0);
}

template<typename T>
OSQPSolution* NMPC::QPProblem<T>::solve(int *error_code) {
    //set up workspace

    if(osqp_workspace_ == nullptr) {
        osqp_settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        osqp_data_     = (OSQPData *)    c_malloc(sizeof(OSQPData));

        //populate data
        osqp_data_->n = n_;
        osqp_data_->m = m_;

        osqp_data_->A = A_.toOSQPCSC();
        osqp_data_->l = l_.data();
        osqp_data_->u = u_.data();

        osqp_data_->P = P_.toOSQPCSC();
        osqp_data_->q = q_.data();



        osqp_set_default_settings(osqp_settings_);
        osqp_setup(&osqp_workspace_, osqp_data_, osqp_settings_);

    }
    else {
        csc *A_csc = A_.toOSQPCSC();
        osqp_update_A(osqp_workspace_, A_csc->x, NULL, A_csc->nzmax);
        osqp_update_bounds(osqp_workspace_, l_.data(), u_.data());

        csc *P_csc = P_.toOSQPCSC();
        osqp_update_P(osqp_workspace_, P_csc->x, NULL, P_csc->nzmax);
        osqp_update_lin_cost(osqp_workspace_, q_.data());
    }

    *error_code = osqp_solve(osqp_workspace_);

    return osqp_workspace_->solution;
}

NMPC::Controller::Controller() {

}

void NMPC::Controller::initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights) {
    parameters_ = parameters;
    model_ = model;

    constraint_ = constraint;
    weights_    = weights;
}

void NMPC::Controller::update(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, const std::vector<double> &curvature_cur, ControlOutput *out, std::vector<State> *pred_out)
{
  const int dim_state = 4;
  const int dim_control = 1;   //四个状态量，一个控制量

  //用于矩阵查找位置的子函数
  auto state_var_idx = [&] (int t, int id){
    return t * dim_state + id;
  };
  auto control_var_idx = [&] (int t, int id){
    return parameters_.pred_horizon * dim_state + std::min(t, parameters_.control_horizon - 1)* dim_control + id;
  };
  auto control_derivative_row_idx = [&] (int t, int id) {
      return dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon + std::min(t, parameters_.control_horizon - 1) * dim_control + id;
  };
  auto state_const_idx_h = [&] (int t, int id) {
      return dim_state * parameters_.pred_horizon + 2*dim_control * parameters_.control_horizon + std::min(t, parameters_.pred_horizon - 1) * 2 + id;
  };
  auto q_slack =[&] (int t, int id){
    return dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon + std::min(t, parameters_.pred_horizon - 1) * 2 + id;
  };


  int n = dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon+2*parameters_.pred_horizon;   //列数，4*NP+1*NC+2NP
  int m = n + dim_control * parameters_.control_horizon;   //行数，4*NP+1*NC+1*NC+2*NP+NP

  c_float dt = parameters_.dt;
  qp_.initialize(n, m);   //84行initialize函数

  //非线性系统线性化雅可比矩阵matlab处理结果
  //gradex_f
  double t2 = std::cos(linearize_point.steer_angle);
  double t3 = std::cos(linearize_point.e_yaw);
  double t4 = std::sin(linearize_point.e_yaw);
  double t5 = model_.l_f * linearize_point.yaw_rate;
  double t6 = model_.l_r * linearize_point.yaw_rate;
  double t8 = pow(model_.B_f, 2.0);
  double t9 = pow(model_.B_r, 2.0);
  double t10 = 1.0 /model_.I_z;
  double t11 = 1.0/model_.m;
  double t14 = -t6;
  double t15 = t5 + linearize_point.v_y;
  double t19 = pow((t6-linearize_point.v_y),2.0);
  double t17 = pow(t15,2.0);
  double t18 = t14+linearize_point.v_y;

  //gradu_f
  double tu2 = std::cos(linearize_point.steer_angle);
  double tu3 = std::sin(linearize_point.steer_angle);
  double tu4 = model_.l_f * linearize_point.yaw_rate;
  double tu5 = pow(model_.B_f,2.0);
  double tu7 = tu4+linearize_point.v_y;



  for(int i = 0; i < dim_state; i++) qp_.A_.addElement(state_var_idx(0, i), state_var_idx(0, i), 1);
  qp_.l_[0] = qp_.u_[0] = state.yaw_rate;
  qp_.l_[1] = qp_.u_[1] = state.v_y;
  qp_.l_[2] = qp_.u_[2] = state.e_yaw;
  qp_.l_[3] = qp_.u_[3] = state.e_d;


  //非线性动力学方程
  for(int t = 1; t < parameters_.pred_horizon; t++){

    double t7 = linearize_point.e_d * curvature_cur[t-1];
    double t12 = 1.0 / track_input[t - 1].v;
    double t13 = pow(t12,2.0);
    double t26 = t13 *t19;
    double t33 = t26+1.0;
    double t38 = 1.0 /t33;
    double t24 = t13 *t17;
    double t27 = t24+1.0;
    double t35 = 1.0 /t27;
    double t16 = t7-1.0;
    double t20 = 1.0 /t16;
    double t21 = t12 *t15;
    double t22 = atan(t21);
    double t28 = -t22;
    double t32 = linearize_point.steer_angle+t28;
    double t36 = pow(t32,2.0);
    double t43 = t8 *t36;
    double t46 = t43+1.0;
    double t53 = 1.0 /t46;
    double t37 = model_.B_f *t32;
    double t39 = atan(t37);
    double t41 = -t37;
    double t57 = t39+t41;
    double t58 = -model_.E_f *(t37-t39);
    double t74 = t37+t58;
    double t75 = atan(t74);
    double t77 = model_.C_f *t75;
    double t78 = cos(t77);
    double t76 = pow(t74,2.0);
    double t79 = t76+1.0;
    double t80 = 1.0 /t79;
    double t23 = -t12 *(t6-linearize_point.v_y);
    double t25 = atan(t23);
    double t29 = model_.B_r *t25;
    double t31 = atan(t29);
    double t34 = -t29;
    double t55 = t31+t34;
    double t56 = -model_.E_r *(t29-t31);
    double t64 = t29+t56;
    double t67 = atan(t64);
    double t70 = model_.C_r *t67;
    double t72 = cos(t70);
    double t69 = pow(t64,2.0);
    double t71 = t69+1.0;
    double t73 = 1.0 /t71;
    double t30 = pow(t25,2.0);
    double t40 = t9 *t30;
    double t42 = t40+1.0;
    double t47 = 1.0 /t42;
    double t49 = model_.B_r *t12 *t38;
    double t51 = model_.l_r *t49;
    double t54 = -t51;
    double t66 = t47 *t54;
    double t61 = t47 *t51;
    double t85 = t51+t66;
    double t88 = model_.E_r *t85;
    double t92 = t54+t88;
    double t52 = -t49;
    double t62 = t47 *t52;
    double t59 = t47 *t49;
    double t82 = t49+t62;
    double t86 = model_.E_r *t82;
    double t90 = t52+t86;
    double t44 = model_.B_f *t12 *t35;
    double t45 = model_.l_f *t44;
    double t50 = -t45;
    double t68 = t50 *t53;
    double t63 = t45 *t53;
    double t83 = t45+t68;
    double t87 = model_.E_f *t83;
    double t91 = t50+t87;
    double t48 = -t44;
    double t65 = t48 *t53;
    double t60 = t44 *t53;
    double t81 = t44+t65;
    double t84 = model_.E_f *t81;
    double t89 = t48+t84;

    double tu6 = 1.0 / track_input[t - 1].v;
    double tu8 = tu6 *tu7;
    double tu9 = atan(tu8);
    double tu10 = -tu9;
    double tu11 = linearize_point.steer_angle +tu10;
    double tu12 = pow(tu11,2.0);
    double tu13 = model_.B_f *tu11;
    double tu14 = atan(tu13);
    double tu15 = -tu13;
    double tu16 = tu5 *tu12;
    double tu17 = tu16+1.0;
    double tu25 = tu14+tu15;
    double tu26 = -model_.E_f *(tu13-tu14);
    double tu18 = 1.0 /tu17;
    double tu27 = tu13+tu26;
    double tu19 = model_.B_f *tu18;
    double tu28 = atan(tu27);
    double tu29 = pow(tu27,2.0);
    double tu20 = -tu19;
    double tu30 = model_.C_f *tu28;
    double tu33 = tu29+1.0;
    double tu21 = model_.B_f+tu20;
    double tu31 = cos(tu30);
    double tu32 = sin(tu30);
    double tu34 = 1.0 /tu33;
    double tu22 = model_.E_f *tu21;
    double tu23 = -tu22;
    double tu24 = model_.B_f+tu23;



    //Xrdot
    double Fzf = model_.m *(model_.g * model_.l_r - model_.h * linearize_point.yaw_rate * linearize_point.v_y) / model_.l;
    double Fzr = model_.m *(model_.g * model_.l_f + model_.h * linearize_point.yaw_rate * linearize_point.v_y) / model_.l;
    double af = -(atan((linearize_point.v_y + linearize_point.yaw_rate * model_.l_f) / track_input[1].v) - linearize_point.steer_angle);
    double ar = -atan((linearize_point.v_y - linearize_point.yaw_rate * model_.l_r) / track_input[1].v);
    double Fyf = model_.D_f*sin(model_.C_f*atan(model_.B_f*af-model_.E_f*(model_.B_f*af -atan(model_.B_f*af))));
    double Fyr = model_.D_r*sin(model_.C_r*atan(model_.B_r*ar-model_.E_r*(model_.B_r*ar -atan(model_.B_r*ar))));

    double yaw_rate_r_dot = (model_.l_f * Fyf * std::cos(linearize_point.steer_angle) - model_.l_r * Fyr) / model_.I_z;
    double v_y_r_dot = (Fyf * std::cos(linearize_point.steer_angle) + Fyr) / model_.m - track_input[1].v * linearize_point.yaw_rate;
    double e_yaw_r_dot = linearize_point.yaw_rate - curvature_cur[1] * (track_input[1].v * std::cos(linearize_point.e_yaw)- linearize_point.v_y * std::sin(linearize_point.e_yaw)) / (1 - linearize_point.e_d * curvature_cur[1]);
    double e_d_r_dot = linearize_point.v_y + track_input[1].v * linearize_point.e_yaw;

    //x雅可比矩阵各项,4*4矩阵
    double ax11 = -t10 *(model_.C_r *model_.D_r *model_.l_r *t72 *t73 *(t51-t88)+model_.C_f *model_.D_f *model_.l_f *t2 *t78 *t80 *(t45-t87));
    double ax12 = t10 *(model_.C_r *model_.D_r *model_.l_r *t72 *t73 *(t49-t86)-model_.C_f *model_.D_f *model_.l_f *t2 *t78 *t80 *(t44-t84));
    double ax13 = 0.0;
    double ax14 = 0.0;
    double ax21 = -track_input[t - 1].v+t11 *(model_.C_r *model_.D_r *t72 *t73 *(t51-t88)-model_.C_f *model_.D_f *t2 *t78 *t80 *(t45-t87));
    double ax22 = -t11 *(model_.C_r *model_.D_r *t72 *t73 *(t49-t86)+model_.C_f *model_.D_f *t2 *t78 *t80 *(t44-t84));
    double ax23 = 0.0;
    double ax24 = 0.0;
    double ax31 = 1.0;
    double ax32 = -curvature_cur[t-1] *t4 *t20;
    double ax33 = -curvature_cur[t-1] *t20 *(t4 *track_input[t - 1].v +t3 *linearize_point.v_y);
    double ax34 = -pow(curvature_cur[t-1], 2) *pow(t20, 2) *(t3 *track_input[t - 1].v- t4 *linearize_point.v_y);
    double ax41 = 0.0;
    double ax42 = 1.0;
    double ax43 = track_input[t - 1].v;
    double ax44 = 0.0;
    //cout << "track_input[t-1].v = " << track_input[t - 1].v <<endl;

    //u雅可比矩阵各项，
    double au11 = -(model_.D_f *model_.l_f *tu3 *tu32- model_.C_f *model_.D_f *model_.l_f *tu2 *tu24 *tu31 *tu34) /model_.I_z;
    double au21 = -(model_.D_f *tu3 *tu32-model_.C_f *model_.D_f *tu2 *tu24 *tu31 *tu34) /model_.m;
    double au31 = 0.0;
    double au41 = 0.0;
    //cout << "ax11 = "<<ax11<<" ax12 = "<<ax12<<" ax21 = " << ax21<< "ax22 = "<<ax22<<" ax32 = "<< ax32<< " ax33 = "<<ax33<<" ax34 = "<< ax34 << endl;
    //cout << "ax33"<<ax33<<endl;
    //cout << "ax34"<<ax34<<endl;
    //cout << "ax43"<<ax43<<endl;
/* cout << "ax11"<<ax11<<endl;
    //求稳定值
    //右侧四项
    double y11 = - yaw_rate_r_dot + ax11 * linearize_point.yaw_rate + ax12 * linearize_point.v_y + au11 * linearize_point.steer_angle;
    double y21 = - v_y_r_dot + ax21 * linearize_point.yaw_rate + ax22 * linearize_point.v_y + au21 * linearize_point.steer_angle;
    double y31 = - e_yaw_r_dot + linearize_point.yaw_rate + ax32 * linearize_point.v_y + ax33 * linearize_point.e_yaw + ax34 * linearize_point.e_d;
    double y41 = - e_d_r_dot + linearize_point.v_y + model_.vx * linearize_point.e_yaw;
    Eigen::Matrix<double,4,1> y_ref;
    y_ref << y11,y21,y31,y41;
    //左侧矩阵
    Eigen::Matrix<double,4,4> a_ref;
    a_ref << ax11,ax12,0,au11,
             ax21,ax22,0,au21,
             1,ax32,ax33,0,
             0,1,model_.vx,0;
    Eigen::MatrixXd x_ref(4,1);
    x_ref = a_ref.inverse() * y_ref;
    double r_stable = x_ref(0,0);
    double v_y_stable = x_ref(1,0);
    double e_yaw_stable = x_ref(2,0);
    double steer_stable = x_ref(3,0);
*/
    //yaw_rate
    qp_.A_.addElement(state_var_idx(t,0), state_var_idx(t, 0), -1.0);   //-x_k+1
    qp_.A_.addElement(state_var_idx(t,0), state_var_idx(t-1, 0), 1.0 + ax11 *dt);  //x_k

    qp_.A_.addElement(state_var_idx(t,0), state_var_idx(t-1, 1), dt * ax12);
    qp_.A_.addElement(state_var_idx(t,0), state_var_idx(t-1, 2), dt * ax13);
    qp_.A_.addElement(state_var_idx(t,0), state_var_idx(t-1, 3), dt * ax14);
    qp_.A_.addElement(state_var_idx(t, 0), control_var_idx(t - 1, 0), au11 *dt);
    qp_.l_[state_var_idx(t, 0)] = qp_.u_[state_var_idx(t, 0)] = -(dt*(yaw_rate_r_dot - linearize_point.yaw_rate * ax11 - ax12 * linearize_point.v_y -ax13 * linearize_point.e_yaw- ax14 * linearize_point.e_d - linearize_point.steer_angle * au11));

    //v_y
    qp_.A_.addElement(state_var_idx(t,1), state_var_idx(t, 1), -1.0);   //-x_k+1
    qp_.A_.addElement(state_var_idx(t,1), state_var_idx(t-1, 0), ax21 *dt);  //x_k

    qp_.A_.addElement(state_var_idx(t,1), state_var_idx(t-1, 1), 1.0+ dt * ax22);
    qp_.A_.addElement(state_var_idx(t,1), state_var_idx(t-1, 2), dt * ax23);
    qp_.A_.addElement(state_var_idx(t,1), state_var_idx(t-1, 3), dt * ax24);
    qp_.A_.addElement(state_var_idx(t, 1), control_var_idx(t - 1, 0), au21 *dt);
    qp_.l_[state_var_idx(t, 1)] = qp_.u_[state_var_idx(t, 1)] = -(dt*(v_y_r_dot - linearize_point.yaw_rate * ax21 - ax22 * linearize_point.v_y -ax23 * linearize_point.e_yaw- ax24 * linearize_point.e_d - linearize_point.steer_angle * au21));

    //e_yaw
    qp_.A_.addElement(state_var_idx(t,2), state_var_idx(t, 2), -1.0);   //-x_k+1
    qp_.A_.addElement(state_var_idx(t,2), state_var_idx(t-1, 0), ax31 *dt);  //x_k

    qp_.A_.addElement(state_var_idx(t,2), state_var_idx(t-1, 1), dt * ax32);
    qp_.A_.addElement(state_var_idx(t,2), state_var_idx(t-1, 2), 1.0+ dt * ax33);
    qp_.A_.addElement(state_var_idx(t,2), state_var_idx(t-1, 3), dt * ax34);
    qp_.A_.addElement(state_var_idx(t, 2), control_var_idx(t - 1, 0), au31 *dt);
    qp_.l_[state_var_idx(t, 2)] = qp_.u_[state_var_idx(t, 2)] = -(dt*(e_yaw_r_dot - linearize_point.yaw_rate * ax31 - ax32 * linearize_point.v_y -ax33 * linearize_point.e_yaw- ax34 * linearize_point.e_d - linearize_point.steer_angle * au31));


    //e_d
    qp_.A_.addElement(state_var_idx(t,3), state_var_idx(t, 3), -1.0);   //-x_k+1
    qp_.A_.addElement(state_var_idx(t,3), state_var_idx(t-1, 0), ax41 *dt);  //x_k

    qp_.A_.addElement(state_var_idx(t,3), state_var_idx(t-1, 1), dt * ax42);
    qp_.A_.addElement(state_var_idx(t,3), state_var_idx(t-1, 2), dt * ax43);
    qp_.A_.addElement(state_var_idx(t,3), state_var_idx(t-1, 3), 1.0+ dt * ax44);
    qp_.A_.addElement(state_var_idx(t, 3), control_var_idx(t - 1, 0), au41 *dt);
    qp_.l_[state_var_idx(t, 3)] = qp_.u_[state_var_idx(t, 3)] = -(dt*(e_d_r_dot - linearize_point.yaw_rate * ax41 - ax42 * linearize_point.v_y -ax43 * linearize_point.e_yaw- ax44 * linearize_point.e_d - linearize_point.steer_angle * au41));
  }
    for(int t = 0; t < parameters_.pred_horizon; t++){
    //建立cost_function
        qp_.P_.addElement(state_var_idx(t, 0), state_var_idx(t, 0), weights_.w_r);
        //qp_.q_[state_var_idx(t, 0)] = -weights_.w_r * r_stable;
        qp_.P_.addElement(state_var_idx(t, 1), state_var_idx(t, 1), weights_.w_v_y);
        //qp_q_[state_var_idx(t, 1)] = -weights_.w_v_y * v_y_stable;
        qp_.P_.addElement(state_var_idx(t, 2), state_var_idx(t, 2), weights_.w_e_yaw);
        //qp_.q_[state_var_idx(t, 2)] = -weights_.w_e_yaw * e_yaw_stable;
        qp_.P_.addElement(state_var_idx(t, 3), state_var_idx(t, 3), weights_.w_e_d);
        //if (t <= parameters_.control_horizon){
          //qp_.q_[control_var_idx(t-1, 0)] = -weights_.w_steer_angle * steer_stable;
        //}
        qp_.P_.addElement(q_slack(t, 0), q_slack(t, 0), weights_.w_slack_eyaw);
        qp_.P_.addElement(q_slack(t, 1), q_slack(t, 1), weights_.w_slack_ey);

  }
    //qp_.P_.addElement(control_var_idx(0, 0), control_var_idx(0, 0), weights_.w_steer_angle);
    //qp_.P_.addElement(control_var_idx(0, 0), control_var_idx(1, 0), -weights_.w_steer_angle);
    //qp_.P_.addElement(control_var_idx(parameters_.control_horizon - 1, 0), control_var_idx(parameters_.control_horizon - 1, 0), weights_.w_steer_angle);
    //qp_.P_.addElement(control_var_idx(parameters_.control_horizon - 1, 0), control_var_idx(parameters_.control_horizon - 2, 0), -weights_.w_steer_angle);
  for (int t = 0; t < parameters_.control_horizon; t++) {
    qp_.P_.addElement(control_var_idx(t, 0), control_var_idx(t, 0),weights_.w_steer_angle);
    //qp_.P_.addElement(control_var_idx(t, 0), control_var_idx(t - 1, 0),-weights_.w_steer_angle);
    //qp_.P_.addElement(control_var_idx(t, 0), control_var_idx(t + 1, 0),-weights_.w_steer_angle);
  }

  //状态量ey,eyaw约束
  for(int t = 0;t < parameters_.pred_horizon;t++){
    for(int i = 0; i < 2;i++) qp_.A_.addElement(state_const_idx_h(t,i), state_var_idx(t,2+i), 1.0);
    for(int i = 0; i < 2;i++) qp_.A_.addElement(state_const_idx_h(t,i), q_slack(t,i), 1.0);
    qp_.l_[state_const_idx_h(t,0)] = -constraint_.max_eyaw;
    qp_.u_[state_const_idx_h(t,0)] = constraint_.max_eyaw;
    qp_.l_[state_const_idx_h(t,1)] = -constraint_.max_ey;
    qp_.u_[state_const_idx_h(t,1)] = constraint_.max_ey;
  }

  //控制量输出约束
  for(int t = 0;t < parameters_.control_horizon;t++){
    for(int i = 0; i < dim_control;i++) qp_.A_.addElement(control_var_idx(t,i), control_var_idx(t,i), 1.0);
    qp_.l_[control_var_idx(t, 0)] = - constraint_.max_steer;
    qp_.u_[control_var_idx(t, 0)] = constraint_.max_steer;
  }

  //控制量变化量约束
  //cout << constraint_.max_steer_rate<<endl;
  double max_delta_steer_angle = dt * constraint_.max_steer_rate;
  //cout <<"delta_steer" <<max_delta_steer_angle<< endl;
  for (int i = 0; i < dim_control;i++){
    qp_.A_.addElement(control_derivative_row_idx(0, i), control_var_idx(0, i), 1.0);
  }

  qp_.l_[control_derivative_row_idx(0, 0)] = state.steer_angle - max_delta_steer_angle;
  qp_.u_[control_derivative_row_idx(0, 0)] = state.steer_angle + max_delta_steer_angle;

  for(int t = 1 ;t < parameters_.control_horizon; t++){
    for(int i = 0; i < dim_control; i++){
      qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t, i), 1.0);
      qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t - 1, i), -1.0);
    }

    qp_.l_[control_derivative_row_idx(t, 0)] = -max_delta_steer_angle;
    qp_.u_[control_derivative_row_idx(t, 0)] = max_delta_steer_angle;
  }

  //输出A矩阵
  //qp_.A_.ShowFun(m, n);
  //qp_.P_.ShowFun(n, n);
  //求解osqp

  OSQPSolution *solution = qp_.solve(&out->error_code);
  out->steer = solution->x[control_var_idx(0, 0)];
  //for (int i = 0;i<parameters_.control_horizon;i++){
    //cout << "steer "<<solution->x[control_var_idx(i,0)];
  //}
  //cout << "the last_eyaw = "<< solution->x[state_var_idx(parameters_.pred_horizon-1,2)];

  //输出预测结果
  if(pred_out != nullptr) {
      pred_out->clear();

      for(int t = 0; t < parameters_.pred_horizon; t++) {
          State pred_state;
          pred_state.yaw_rate = solution->x[state_var_idx(t, 0)];
          pred_state.v_y = solution->x[state_var_idx(t, 1)];
          pred_state.e_yaw = solution->x[state_var_idx(t, 2)];
          pred_state.e_d = solution->x[state_var_idx(t, 3)];

          pred_out->push_back(pred_state);    //获取状态量预测
      }

}
}

//计算线性化参考点
void NMPC::IterativeController::simulate(const State &state, const std::vector<double> &curvature_cur ,State *next) {
    double dt = parameters_.dt;
    double Fzf = model_.m *(model_.g * model_.l_r - model_.h * state.yaw_rate * state.v_y) / model_.l;
    double Fzr = model_.m *(model_.g * model_.l_f + model_.h * state.yaw_rate * state.v_y) / model_.l;

    double af = -(atan((state.v_y + state.yaw_rate * model_.l_f) / state.v_x) - state.steer_angle);
    double ar = -atan((state.v_y - state.yaw_rate * model_.l_r) / state.v_x);
    //cout << "alpha= "<<af <<" "<<ar<<endl;
    double Fyf = model_.D_f*sin(model_.C_f*atan(model_.B_f*af-model_.E_f*(model_.B_f*af -atan(model_.B_f*af))));
    double Fyr = model_.D_r*sin(model_.C_r*atan(model_.B_r*ar-model_.E_r*(model_.B_r*ar -atan(model_.B_r*ar))));
    //cout << "Fy= "<<Fyf << "  "<<Fyr <<endl;

    next->yaw_rate = state.yaw_rate + (model_.l_f * Fyf * std::cos(state.steer_angle) - model_.l_r * Fyr) / model_.I_z * dt;
    next->v_y = state.v_y + ((Fyf * std::cos(state.steer_angle) + Fyr) / model_.m - state.v_x * state.yaw_rate) * dt;
    next->e_yaw = state.e_yaw + (state.yaw_rate - curvature_cur[0] * (state.v_x * std::cos(state.e_yaw)- state.v_y * std::sin(state.e_yaw)) / (1 - state.e_d * curvature_cur[0])) * dt;
    next->e_d = state.e_d + (state.v_y + state.v_x * state.e_yaw) * dt;

    next->steer_angle = state.steer_angle;
}

void NMPC::IterativeController::update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold,const std::vector<double> &curvature_cur ,ControlOutput *out, std::vector<State> *pred_out) {
  State state_with_new_control_input = state;

  int num_iterations = 0;
  for(int i = 0; i < 1; i++){
    num_iterations++;

    State linearize_point;
    //cout << state_with_new_control_input.yaw_rate << "  "<<state_with_new_control_input.v_y<<"  "<<state_with_new_control_input.e_yaw<<"  "<<state_with_new_control_input.e_d<<"  "<<state_with_new_control_input.steer_angle<< endl;
    simulate(state_with_new_control_input, curvature_cur ,&linearize_point);
    //cout << "linearize"<< linearize_point.yaw_rate<<" "<<linearize_point.v_y<<" "<<linearize_point.e_yaw<<" "<<linearize_point.e_d<<endl;
    Controller::update(state, linearize_point, track_input, curvature_cur ,out, pred_out);


    //判断迭代是否应该终止
    double delta_output = std::fabs(state_with_new_control_input.steer_angle - out->steer);
    if(delta_output < threshold) break;
    state_with_new_control_input.steer_angle = out->steer;

    //可视化控制量
    //cout << "steer_angle = "<<state_with_new_control_input.steer_angle <<endl;
  }
   //cout << "steer_angle = "<<state_with_new_control_input.steer_angle <<endl;
printf("%d iterations\n", num_iterations);
}



