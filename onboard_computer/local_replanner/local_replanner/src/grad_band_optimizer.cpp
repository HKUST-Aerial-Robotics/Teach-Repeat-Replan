#include "grad_replanner/grad_band_optimizer.h"
#include <nlopt.hpp>
using namespace std;

class ConstrainData {
 private:
  /* data */
 public:
  ConstrainData(Eigen::Vector3d fp, Eigen::Vector3d lp, int pn, int vn, int id,
                int a, int s)
      : first_pt(fp),
        last_pt(lp),
        point_num(pn),
        variable_num(vn),
        idx(id),
        axis(a),
        sign(s) {
    // show();
  }
  ~ConstrainData(){};

  void show() {
    cout << "cons data:\n"
         << first_pt.transpose() << ", " << last_pt.transpose() << "\n"
         << point_num << ", " << variable_num << ", " << idx << ", " << axis
         << ", " << sign << endl;
  }

  Eigen::Vector3d first_pt, last_pt;
  int point_num, variable_num;
  int idx, axis, sign;
};

void GradBandOptimizer::setControlPoints(Eigen::MatrixXd points) {
  this->control_points = points;
  this->start_id = order;
  this->end_id = this->control_points.rows() - order;
  this->lamda1 = this->lamda1_init;
}

void GradBandOptimizer::setOptimizationRange(int start, int end) {
  this->start_id = min(max(start, order), int(control_points.rows() - order));
  this->end_id = min(max(end, order), int(control_points.rows() - order));
  // cout << "opt range:" << this->start_id << ", " << this->end_id << endl;
}

void GradBandOptimizer::setParameterAuto(double max_v, double max_a) {
  ros::param::get("/local_replanner/lamda1", lamda1_init);
  ros::param::get("/local_replanner/lamda2", lamda2);
  ros::param::get("/local_replanner/lamda3", lamda3);
  ros::param::get("/local_replanner/lamda4", lamda4);
  ros::param::get("/local_replanner/alpha", alpha);
  ros::param::get("/local_replanner/beta", beta);
  ros::param::get("/local_replanner/dist0", dist0);
  ros::param::get("/local_replanner/max_iteration_num", max_iteration_num);
  ros::param::get("/local_replanner/algorithm", algorithm);
/*  ros::param::get("/local_replanner/max_vel", max_vel);
  ros::param::get("/local_replanner/max_acc", max_acc);*/
  ros::param::get("/local_replanner/order", order);

  max_vel = max_v;
  max_acc = max_a;

  cout<<"MAX Velocity := "    <<max_vel<<endl;
  cout<<"MAX Acceleration := "<<max_acc<<endl;
}

/*void GradBandOptimizer::setParameter(double lamda1_, double lamda2_, double
lamda3_, double max_vel_, double max_acc_) { lamda1_init = lamda1_; lamda2  =
lamda2_; lamda3  = lamda3_; max_vel = max_vel_; max_acc = max_acc_;
}*/

void GradBandOptimizer::renewLambda1(double lamda1_) { lamda1_init += lamda1_; }

void GradBandOptimizer::renewLambda2(double lamda2_) { lamda2 += lamda2_; }

void GradBandOptimizer::resetLambda2()
{
  ros::param::get("/local_replanner/lamda2", lamda2);
//  cout << "reset lamda2: " << lamda2 << endl;
}

void GradBandOptimizer::setBSplineInterval(double ts) {
  this->bspline_interval = ts;
}

void GradBandOptimizer::setDistanceField(const shared_ptr<SDFMap>& map) {
  sdf_map = map;
}

void GradBandOptimizer::getDistanceAndGradient(Eigen::Vector3d& pos,
                                               double& dist,
                                               Eigen::Vector3d& grad) {
  dist = sdf_map->getDistWithGradTrilinear(pos, grad);
}

Eigen::MatrixXd GradBandOptimizer::getControlPoints() {
  return this->control_points;
}

/* best algorithm is 40: SLSQP(constrained), 11 LBFGS(unconstrained barrier
method */
void GradBandOptimizer::optimizeTrajFixedEnd() {

  // this->variable_num = 3 * (control_points.rows() - 2 * order);
  this->variable_num = 3 * (end_id - start_id);
  this->min_cost = std::numeric_limits<double>::max();
  nlopt::opt opt(nlopt::algorithm(algorithm), this->variable_num);
  // opt.set_min_objective(GradBandOptimizer::costFuncMinCurvaturePara, this);
  opt.set_min_objective(GradBandOptimizer::costFuncMinJerkPara, this);
  opt.set_maxeval(max_iteration_num);
  // opt.set_maxtime(1e-2);
  // opt.set_xtol_rel(1e-4);

  vector<double> q(this->variable_num);
  double final_cost;
  for (int i = 0; i < int(control_points.rows()); ++i) {
    if (i < start_id || i >= end_id) continue;
    q[0 + 3 * (i - start_id)] = control_points(i, 0);
    q[1 + 3 * (i - start_id)] = control_points(i, 1);
    q[2 + 3 * (i - start_id)] = control_points(i, 2);
  }

  try {
    // SETCY << "[gradient elastic-band]:begin-------------" << REC;
    cout << fixed << setprecision(7);

    vec_time.clear();
    vec_cost.clear();
    time_start = ros::Time::now();
    nlopt::result result = opt.optimize(q, final_cost);

    // SETCY << "Min cost:" << min_cost << REC;
    for (int i = 0; i < control_points.rows(); ++i) {
      if (i < start_id || i >= end_id) continue;
      control_points(i, 0) = this->best_variable[0 + 3 * (i - start_id)];
      control_points(i, 1) = this->best_variable[1 + 3 * (i - start_id)];
      control_points(i, 2) = this->best_variable[2 + 3 * (i - start_id)];
    }

    // SETCY << "[gradient elastic-band]:end-------------" << REC;
  } catch (std::exception& e) {
    SETR << "[Grad band]: nlopt fail: " << e.what() << REC;
  }

  // /* add contrains */
  // vector<ConstrainData*> cons;
  // for (int idx = 0; idx <= this->control_points.rows() - 2; ++idx) {
  //   for (int axis = 0; axis <= 2; ++axis) {
  //     for (int sign = -1; sign <= 1; ++sign) {
  //       if (sign == 0) continue;

  //       ConstrainData* con = new ConstrainData(
  //           control_points.row(0).transpose(),
  //           control_points.row(control_points.rows() - 1).transpose(),
  //           control_points.rows(), 3 * (control_points.rows() - 2), idx,
  //           axis, sign);
  //       cons.push_back(con);
  //       int con_num = cons.size();
  //       opt.add_inequality_constraint(GradBandOptimizer::velConstraint, con,
  //                                     1e-3);
  //       if (idx <= this->control_points.rows() - 3)
  //         opt.add_inequality_constraint(GradBandOptimizer::accConstraint,
  //         con,
  //                                       1e-3);
  //     }
  //   }
  // }
}

/* end pos and vel is not fixed */
void GradBandOptimizer::optimizeTrajFreeEnd() {
  // this->variable_num = 3 * (control_points.rows() - 2 * order);
  this->variable_num = 3 * (control_points.rows() - order);
  this->min_cost = std::numeric_limits<double>::max();
  nlopt::opt opt(nlopt::algorithm(algorithm), this->variable_num);
  opt.set_min_objective(GradBandOptimizer::costFuncMinCurvatureParaFreeEnd,
                        this);
  opt.set_maxeval(max_iteration_num);
  // opt.set_maxtime(1e-2);
  // opt.set_xtol_rel(1e-4);

  vector<double> q(this->variable_num);
  double final_cost;
  for (int i = 0; i < int(control_points.rows()); ++i) {
    if (i < order) continue;
    q[0 + 3 * (i - order)] = control_points(i, 0);
    q[1 + 3 * (i - order)] = control_points(i, 1);
    q[2 + 3 * (i - order)] = control_points(i, 2);
  }

  end_pt = (1 / 6.0) * (control_points.row(control_points.rows() - 3) +
                        4 * control_points.row(control_points.rows() - 2) +
                        control_points.row(control_points.rows() - 1));

  try {
    SETCY << "[Grad band]: begin (free end) ------" << REC;
    cout << fixed << setprecision(7);

    nlopt::result result = opt.optimize(q, final_cost);

    SETCY << "Min cost:" << min_cost << REC;
    for (int i = 0; i < control_points.rows(); ++i) {
      if (i < order) continue;
      control_points(i, 0) = this->best_variable[0 + 3 * (i - order)];
      control_points(i, 1) = this->best_variable[1 + 3 * (i - order)];
      control_points(i, 2) = this->best_variable[2 + 3 * (i - order)];
    }

    SETCY << "[Grad band]: end-------------" << REC;
  } catch (std::exception& e) {
    SETR << "[Grad band]: nlopt fail: " << e.what() << REC;
  }
}

double GradBandOptimizer::costFuncMinCurvaturePara(const std::vector<double>& x,
                                                   std::vector<double>& grad,
                                                   void* func_data) {
  GradBandOptimizer* opt = reinterpret_cast<GradBandOptimizer*>(func_data);
  grad.resize(opt->variable_num);
  for (int i = 0; i < grad.size(); ++i) {
    grad[i] = 0;
  }
  static int optnum = 0;
  ++optnum;

  /* optimize unfixed control points of bspline */
  double f_combine = 0.0, f_curvature = 0.0, f_exp_vel = 0.0, f_exp_acc = 0.0,
         f_dist = 0.0;
  for (int i = 0; i < opt->control_points.rows(); ++i) {
    if (i < opt->start_id || i >= opt->end_id) continue;

    /* get the current, back and front control_points */
    Eigen::Vector3d q1, q2, q3;
    if (i == opt->start_id)
      q1 = opt->control_points.row(opt->start_id - 1);
    else {
      q1(0) = x[0 + 3 * (i - opt->start_id - 1)];
      q1(1) = x[1 + 3 * (i - opt->start_id - 1)];
      q1(2) = x[2 + 3 * (i - opt->start_id - 1)];
    }

    q2(0) = x[0 + 3 * (i - opt->start_id)];
    q2(1) = x[1 + 3 * (i - opt->start_id)];
    q2(2) = x[2 + 3 * (i - opt->start_id)];

    if (i == opt->end_id - 1)
      q3 = opt->control_points.row(opt->end_id);
    else {
      q3(0) = x[0 + 3 * ((i - opt->start_id + 1))];
      q3(1) = x[1 + 3 * ((i - opt->start_id + 1))];
      q3(2) = x[2 + 3 * ((i - opt->start_id + 1))];
    }

    /* f_curvature = Sigma ||q1+q3-2q2||^2 */
    double f1 = (q1 + q3 - 2 * q2).squaredNorm();
    Eigen::Vector3d q0, q4;
    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
      f1 += (q0 + q2 - 2 * q1).squaredNorm();
    } else if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      f1 += (q2 + q4 - 2 * q3).squaredNorm();
    }
    f_curvature += f1;

    /* f_dist = (d-d0)^2, if d<d0;  0, if d>=d0 */
    Eigen::Vector3d dist_grad;
    double dist;
    opt->getDistanceAndGradient(q2, dist, dist_grad);
    double f2 = 0.0;
    if (dist < opt->dist0) f2 = pow(dist - opt->dist0, 2);
    f_dist += f2;

    /* only constrain the first few points */
    static double vm = opt->max_vel, am = opt->max_acc,
                  ts = opt->bspline_interval;
    static double vm2 = vm * vm;
    static double am2 = am * am;
    static double ts_inv = 1 / ts;
    Eigen::Vector3d q32;
    Eigen::Vector3d q21;
    Eigen::Vector3d q132;
    Eigen::Vector3d q243;
    Eigen::Vector3d q021;

    q21 = q2 - q1;
    double fv = 0.0;
    double conv21[3] = {0, 0, 0};
    for (int j = 0; j < 3; ++j) {
      conv21[j] = q21(j) * q21(j) * ts_inv * ts_inv - vm2;
      if (conv21[j] > 0.0) fv += pow(conv21[j], 2);
    }
    f_exp_vel += fv;

    double conv32[3] = {0, 0, 0};
    if (i == opt->end_id - 1) {
      q32 = q3 - q2;
      fv = 0.0;
      for (int j = 0; j < 3; ++j) {
        conv32[j] = q32(j) * q32(j) * ts_inv * ts_inv - vm2;
        if (conv32[j] > 0.0) fv += pow(conv32[j], 2);
      }
      f_exp_vel += fv;
    }

    /* f_exp_acc = alpha * exp{beta * [ (q3x+q1x-2 q2x)^2 - (vm*ts^2)^2] }*/
    q132 = q1 + q3 - 2 * q2;
    double fa = 0.0;
    double cona132[3] = {0, 0, 0};
    for (int j = 0; j < 3; ++j) {
      cona132[j] = q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
      if (cona132[j] > 0.0) fa += pow(cona132[j], 2);
    }
    f_exp_acc += fa;

    double cona021[3] = {0, 0, 0};
    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
      q021 = q0 + q2 - 2 * q1;
      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        cona021[j] =
            q021(j) * q021(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona021[j] > 0.0) fa += pow(cona021[j], 2);
      }
      f_exp_acc += fa;
    }

    double cona243[3] = {0, 0, 0};
    if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      q243 = q2 + q4 - 2 * q3;
      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        cona243[j] =
            q243(j) * q243(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona243[j]) fa += pow(cona243[j], 2);
      }
      f_exp_acc += fa;
    }
    // }

    /* gradient of curvature cost */
    Eigen::Vector3d g_cur_q1, g_cur_q2, g_cur_q3;
    g_cur_q1 = g_cur_q3 = 2 * (q1 + q3 - 2 * q2);
    g_cur_q2 = -4 * (q1 + q3 - 2 * q2);

    if (i == opt->start_id) {
      g_cur_q2 += 2 * (q0 + q2 - 2 * q1);
    } else if (i == opt->end_id - 1) {
      g_cur_q2 += 2 * (q2 + q4 - 2 * q3);
    }

    /* gradient of dist cost */
    Eigen::Vector3d g_dist_q2 = Eigen::Vector3d::Zero();
    if (dist < opt->dist0) g_dist_q2 = 2 * (dist - opt->dist0) * dist_grad;

    Eigen::Vector3d g_vel_q1(0, 0, 0), g_vel_q2(0, 0, 0);
    Eigen::Vector3d g_acc_q1(0, 0, 0), g_acc_q2(0, 0, 0), g_acc_q3(0, 0, 0);
    // if (i < opt->order + constrained_num ||
    //     i >= opt->control_points.rows() - opt->order - constrained_num) {
    for (int j = 0; j < 3; ++j) {
      /* gradient of exp vel */
      if (conv21[j] > 0.0) {
        g_vel_q2(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * 2.0 * q21(j);
        g_vel_q1(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * (-2.0) * q21(j);
      }
      if (i == opt->end_id - 1) {
        if (conv32[j] > 0.0)
          g_vel_q2(j) += 2.0 * ts_inv * ts_inv * (conv32[j]) * (-2.0) * q32(j);
      }

      /* gradient of exp acc */
      if (cona132[j] > 0.0) {
        g_acc_q1(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      2.0 * q132(j);
        g_acc_q2(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      (-4.0) * q132(j);
        g_acc_q3(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      2.0 * q132(j);
      }
      if (i == opt->start_id) {
        if (cona021[j] > 0.0)
          g_acc_q2(j) += 2.0 * ts_inv * ts_inv * ts_inv * ts_inv *
                         (cona021[j]) * 2.0 * q021(j);
      }
      if (i == opt->end_id - 1) {
        if (cona243[j] > 0.0)
          g_acc_q2(j) += 2.0 * ts_inv * ts_inv * ts_inv * ts_inv *
                         (cona243[j]) * 2.0 * q243(j);
      }
    }
    // }

    // return gradient of function and cost
    if (i != opt->start_id) {
      grad[0 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(0) + opt->lamda3 * (g_vel_q1(0) + g_acc_q1(0));
      grad[1 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(1) + opt->lamda3 * (g_vel_q1(1) + g_acc_q1(1));
      grad[2 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(2) + opt->lamda3 * (g_vel_q1(2) + g_acc_q1(2));
    }

    grad[0 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(0) + opt->lamda2 * g_dist_q2(0) +
        opt->lamda3 * (g_vel_q2(0) + g_acc_q2(0));
    grad[1 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(1) + opt->lamda2 * g_dist_q2(1) +
        opt->lamda3 * (g_vel_q2(1) + g_acc_q2(1));
    grad[2 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(2) + opt->lamda2 * g_dist_q2(2) +
        opt->lamda3 * (g_vel_q2(2) + g_acc_q2(2));

    if (i != opt->end_id - 1) {
      grad[0 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(0) + opt->lamda3 * g_acc_q3(0);
      grad[1 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(1) + opt->lamda3 * g_acc_q3(1);
      grad[2 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(2) + opt->lamda3 * g_acc_q3(2);
    }
  }

  /* f = l1 * f_curvature + l2 * f_dist + l3 * (f_exp_vel + f_exp_acc)  */
  f_combine = opt->lamda1 * f_curvature + opt->lamda2 * f_dist +
              opt->lamda3 * (f_exp_vel + f_exp_acc);

  if (optnum % 30 == 0) {
    /*SETCY << optnum << " cur: " << opt->lamda1 * f_curvature
          << " , dist: " << opt->lamda2 * f_dist
          << ", exp:" << opt->lamda3 * (f_exp_acc + f_exp_vel)
          << ", total: " << f_combine << REC;*/

    // cout << "average curvature cost: "
    //      << opt->lamda1 * f_curvature / (opt->control_points.rows() - 6)
    //      << "\n avg dist cost: "
    //      << opt->lamda2 * f_dist / (opt->control_points.rows() - 6)
    //      << "\ncontrol points num: " << opt->control_points.rows() - 6 <<
    //      endl;
    // cout << "lamda: " << opt->lamda1 << endl;
  }

  /* evaluation */
  ros::Time te1 = ros::Time::now();
  double time_now = (te1 - opt->time_start).toSec();
  opt->vec_time.push_back(time_now);
  if (opt->vec_cost.size() == 0) {
    opt->vec_cost.push_back(f_combine);
  } else if (opt->vec_cost.back() > f_combine) {
    opt->vec_cost.push_back(f_combine);
  } else {
    opt->vec_cost.push_back(opt->vec_cost.back());
  }

  /* save the min cost result */
  if (f_combine < opt->min_cost) {
    opt->min_cost = f_combine;
    opt->best_variable = x;
  }
  return f_combine;
}

double GradBandOptimizer::costFuncMinJerkPara(const std::vector<double>& x,
                                              std::vector<double>& grad,
                                              void* func_data) {
  GradBandOptimizer* opt = reinterpret_cast<GradBandOptimizer*>(func_data);
  grad.resize(opt->variable_num);
  for (int i = 0; i < grad.size(); ++i) {
    grad[i] = 0;
  }
  static int optnum = 0;
  ++optnum;

  /* optimize unfixed control points of bspline */
  double f_combine = 0.0, f_curvature = 0.0, f_exp_vel = 0.0, f_exp_acc = 0.0,
         f_dist = 0.0;
  for (int i = 0; i < opt->control_points.rows(); ++i) {
    if (i < opt->start_id || i >= opt->end_id) continue;

    /* get the current, back and front control_points */
    Eigen::Vector3d q0, q1, q2, q3;

    /* get q0 */
    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
    } else if (i == opt->start_id + 1) {
      q0 = opt->control_points.row(opt->start_id - 1);
    } else {
      q0(0) = x[0 + 3 * (i - opt->start_id - 2)];
      q0(1) = x[1 + 3 * (i - opt->start_id - 2)];
      q0(2) = x[2 + 3 * (i - opt->start_id - 2)];
    }
    /* get q1 */
    if (i == opt->start_id)
      q1 = opt->control_points.row(opt->start_id - 1);
    else {
      q1(0) = x[0 + 3 * (i - opt->start_id - 1)];
      q1(1) = x[1 + 3 * (i - opt->start_id - 1)];
      q1(2) = x[2 + 3 * (i - opt->start_id - 1)];
    }
    /* get q2 */
    q2(0) = x[0 + 3 * (i - opt->start_id)];
    q2(1) = x[1 + 3 * (i - opt->start_id)];
    q2(2) = x[2 + 3 * (i - opt->start_id)];
    /* get q3 */
    if (i == opt->end_id - 1)
      q3 = opt->control_points.row(opt->end_id);
    else {
      q3(0) = x[0 + 3 * ((i - opt->start_id + 1))];
      q3(1) = x[1 + 3 * ((i - opt->start_id + 1))];
      q3(2) = x[2 + 3 * ((i - opt->start_id + 1))];
    }

    /* f_curvature = Sigma ||q3-3q2+3q1-q0||^2 */
    double f1 = (q3 - 3 * q2 + 3 * q1 - q0).squaredNorm();
    Eigen::Vector3d q_1, q4, q5;
    if (i == opt->start_id) {
      q_1 = opt->control_points.row(opt->start_id - 3);
      f1 += (q2 - 3 * q1 + 3 * q0 - q_1).squaredNorm();
    } else if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      q5 = opt->control_points.row(opt->end_id + 2);
      f1 += (q4 - 3 * q3 + 3 * q2 - q1).squaredNorm();
      f1 += (q5 - 3 * q4 + 3 * q3 - q2).squaredNorm();
    }
    f_curvature += f1;

    /* f_dist = (d-d0)^2, if d<d0;  0, if d>=d0 */
    Eigen::Vector3d dist_grad;
    double dist;
    opt->getDistanceAndGradient(q2, dist, dist_grad);
    double f2 = 0.0;
    if (dist < opt->dist0) f2 = pow(dist - opt->dist0, 2);
    f_dist += f2;

    /* only constrain the first few points */
    static double vm = opt->max_vel, am = opt->max_acc,
                  ts = opt->bspline_interval;
    static double vm2 = vm * vm;
    static double am2 = am * am;
    static double ts_inv = 1 / ts;
    Eigen::Vector3d q32;
    Eigen::Vector3d q21;
    Eigen::Vector3d q132;
    Eigen::Vector3d q243;
    Eigen::Vector3d q021;

    q21 = q2 - q1;
    double fv = 0.0;
    double conv21[3] = {0, 0, 0};
    for (int j = 0; j < 3; ++j) {
      conv21[j] = q21(j) * q21(j) * ts_inv * ts_inv - vm2;
      if (conv21[j] > 0.0) fv += pow(conv21[j], 2);
    }
    f_exp_vel += fv;

    double conv32[3] = {0, 0, 0};
    if (i == opt->end_id - 1) {
      q32 = q3 - q2;
      fv = 0.0;
      for (int j = 0; j < 3; ++j) {
        conv32[j] = q32(j) * q32(j) * ts_inv * ts_inv - vm2;
        if (conv32[j] > 0.0) fv += pow(conv32[j], 2);
      }
      f_exp_vel += fv;
    }

    /* f_exp_acc = alpha * exp{beta * [ (q3x+q1x-2 q2x)^2 - (vm*ts^2)^2] }*/
    q132 = q1 + q3 - 2 * q2;
    double fa = 0.0;
    double cona132[3] = {0, 0, 0};
    for (int j = 0; j < 3; ++j) {
      cona132[j] = q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
      if (cona132[j] > 0.0) fa += pow(cona132[j], 2);
    }
    f_exp_acc += fa;

    double cona021[3] = {0, 0, 0};
    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
      q021 = q0 + q2 - 2 * q1;
      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        cona021[j] =
            q021(j) * q021(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona021[j] > 0.0) fa += pow(cona021[j], 2);
      }
      f_exp_acc += fa;
    }

    double cona243[3] = {0, 0, 0};
    if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      q243 = q2 + q4 - 2 * q3;
      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        cona243[j] =
            q243(j) * q243(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona243[j]) fa += pow(cona243[j], 2);
      }
      f_exp_acc += fa;
    }
    // }

    /* gradient of curvature cost */
    Eigen::Vector3d g_cur_q0, g_cur_q1, g_cur_q2, g_cur_q3;
    g_cur_q0 = -2 * (q3 - 3 * q2 + 3 * q1 - q0);
    g_cur_q1 = 6 * (q3 - 3 * q2 + 3 * q1 - q0);
    g_cur_q2 = -6 * (q3 - 3 * q2 + 3 * q1 - q0);
    g_cur_q3 = 2 * (q3 - 3 * q2 + 3 * q1 - q0);

    if (i == opt->start_id) {
      g_cur_q2 += 2 * (q2 - 3 * q1 + 3 * q0 - q_1);
    } else if (i == opt->end_id - 1) {
      g_cur_q1 += -2 * (q4 - 3 * q3 + 3 * q2 - q1);
      g_cur_q2 += 6 * (q4 - 3 * q3 + 3 * q2 - q1);
      g_cur_q2 += -2 * (q5 - 3 * q4 + 3 * q3 - q2);
    }

    /* gradient of dist cost */
    Eigen::Vector3d g_dist_q2 = Eigen::Vector3d::Zero();
    if (dist < opt->dist0) g_dist_q2 = 2 * (dist - opt->dist0) * dist_grad;

    Eigen::Vector3d g_vel_q1(0, 0, 0), g_vel_q2(0, 0, 0);
    Eigen::Vector3d g_acc_q1(0, 0, 0), g_acc_q2(0, 0, 0), g_acc_q3(0, 0, 0);
    // if (i < opt->order + constrained_num ||
    //     i >= opt->control_points.rows() - opt->order - constrained_num) {
    for (int j = 0; j < 3; ++j) {
      /* gradient of exp vel */
      if (conv21[j] > 0.0) {
        g_vel_q2(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * 2.0 * q21(j);
        g_vel_q1(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * (-2.0) * q21(j);
      }
      if (i == opt->end_id - 1) {
        if (conv32[j] > 0.0)
          g_vel_q2(j) += 2.0 * ts_inv * ts_inv * (conv32[j]) * (-2.0) * q32(j);
      }

      /* gradient of exp acc */
      if (cona132[j] > 0.0) {
        g_acc_q1(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      2.0 * q132(j);
        g_acc_q2(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      (-4.0) * q132(j);
        g_acc_q3(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                      2.0 * q132(j);
      }
      if (i == opt->start_id) {
        if (cona021[j] > 0.0)
          g_acc_q2(j) += 2.0 * ts_inv * ts_inv * ts_inv * ts_inv *
                         (cona021[j]) * 2.0 * q021(j);
      }
      if (i == opt->end_id - 1) {
        if (cona243[j] > 0.0)
          g_acc_q2(j) += 2.0 * ts_inv * ts_inv * ts_inv * ts_inv *
                         (cona243[j]) * 2.0 * q243(j);
      }
    }
    // }

    // return gradient of function and cost
    if (i != opt->start_id && i != opt->start_id + 1) {
      grad[0 + 3 * (i - opt->start_id - 2)] += opt->lamda1 * g_cur_q0(0);
      grad[1 + 3 * (i - opt->start_id - 2)] += opt->lamda1 * g_cur_q0(1);
      grad[2 + 3 * (i - opt->start_id - 2)] += opt->lamda1 * g_cur_q0(2);
    }

    if (i != opt->start_id) {
      grad[0 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(0) + opt->lamda3 * (g_vel_q1(0) + g_acc_q1(0));
      grad[1 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(1) + opt->lamda3 * (g_vel_q1(1) + g_acc_q1(1));
      grad[2 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(2) + opt->lamda3 * (g_vel_q1(2) + g_acc_q1(2));
    }

    grad[0 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(0) + opt->lamda2 * g_dist_q2(0) +
        opt->lamda3 * (g_vel_q2(0) + g_acc_q2(0));
    grad[1 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(1) + opt->lamda2 * g_dist_q2(1) +
        opt->lamda3 * (g_vel_q2(1) + g_acc_q2(1));
    grad[2 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(2) + opt->lamda2 * g_dist_q2(2) +
        opt->lamda3 * (g_vel_q2(2) + g_acc_q2(2));

    if (i != opt->end_id - 1) {
      grad[0 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(0) + opt->lamda3 * g_acc_q3(0);
      grad[1 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(1) + opt->lamda3 * g_acc_q3(1);
      grad[2 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(2) + opt->lamda3 * g_acc_q3(2);
    }
  }

  /* f = l1 * f_curvature + l2 * f_dist + l3 * (f_exp_vel + f_exp_acc)  */
  f_combine = opt->lamda1 * f_curvature + opt->lamda2 * f_dist +
              opt->lamda3 * (f_exp_vel + f_exp_acc);

  // if (optnum % 40 == 0) {
    // cout << optnum << " cur: " << opt->lamda1 * f_curvature
    //       << " , dist: " << opt->lamda2 * f_dist
    //       << ", exp:" << opt->lamda3 * (f_exp_acc + f_exp_vel)
    //       << ", total: " << f_combine << endl;
    // cout << "average curvature cost: "
    //      << opt->lamda1 * f_curvature / (opt->control_points.rows() - 6)
    //      << "\n avg dist cost: "
    //      << opt->lamda2 * f_dist / (opt->control_points.rows() - 6)
    //      << "\ncontrol points num: " << opt->control_points.rows() - 6 <<
    //      endl;
    // cout << "lamda: " << opt->lamda1 << endl;
  // }

  /* evaluation */
  ros::Time te1 = ros::Time::now();
  double time_now = (te1 - opt->time_start).toSec();
  opt->vec_time.push_back(time_now);
  if (opt->vec_cost.size() == 0) {
    opt->vec_cost.push_back(f_combine);
  } else if (opt->vec_cost.back() > f_combine) {
    opt->vec_cost.push_back(f_combine);
  } else {
    opt->vec_cost.push_back(opt->vec_cost.back());
  }

  /* save the min cost result */
  if (f_combine < opt->min_cost) {
    opt->min_cost = f_combine;
    opt->best_variable = x;
  }
  return f_combine;
}

double GradBandOptimizer::costFuncMinCurvatureParaFreeEnd(
    const std::vector<double>& x, std::vector<double>& grad, void* func_data) {
  GradBandOptimizer* opt = reinterpret_cast<GradBandOptimizer*>(func_data);
  grad.resize(opt->variable_num);
  for (int i = 0; i < grad.size(); ++i) {
    grad[i] = 0;
  }
  static int optnum = 0;
  ++optnum;

  /* optimize unfixed control points of bspline */
  double f_combine = 0.0, f_curvature = 0.0, f_exp_vel = 0.0, f_exp_acc = 0.0,
         f_dist = 0.0, f_end = 0.0;
  for (int i = 0; i < opt->control_points.rows(); ++i) {
    if (i < opt->order) continue;

    /* get the current, back and front control_points */
    Eigen::Vector3d q1, q2, q3;
    if (i == opt->order)
      q1 = opt->control_points.row(opt->order - 1);
    else {
      q1(0) = x[0 + 3 * (i - opt->order - 1)];
      q1(1) = x[1 + 3 * (i - opt->order - 1)];
      q1(2) = x[2 + 3 * (i - opt->order - 1)];
    }

    q2(0) = x[0 + 3 * (i - opt->order)];
    q2(1) = x[1 + 3 * (i - opt->order)];
    q2(2) = x[2 + 3 * (i - opt->order)];

    bool last = i == opt->control_points.rows() - 1;

    if (!last) {
      q3(0) = x[0 + 3 * ((i - opt->order + 1))];
      q3(1) = x[1 + 3 * ((i - opt->order + 1))];
      q3(2) = x[2 + 3 * ((i - opt->order + 1))];
    }

    /* add free end point penalty */
    bool last_second = i == opt->control_points.rows() - 2;
    double f_end_penalty;
    Eigen::Vector3d d_to_end;
    if (last_second) {
      d_to_end = (1 / 6.0 * (q1 + 4 * q2 + q3) - opt->end_pt);
      double fe = d_to_end.squaredNorm();
      f_end += fe;
    }

    /* f_curvature = Sigma ||q1+q3-2q2||^2 */
    Eigen::Vector3d q0, q4;
    if (!last) {
      double f1 = (q1 + q3 - 2 * q2).squaredNorm();
      f_curvature += f1;
    }
    if (i == opt->order) {
      q0 = opt->control_points.row(opt->order - 2);
      double f1 = (q0 + q2 - 2 * q1).squaredNorm();
      f_curvature += f1;
    }

    /* f_dist = (d-d0)^2, if d<d0;  0, if d>=d0 */
    Eigen::Vector3d dist_grad;
    double dist;
    opt->getDistanceAndGradient(q2, dist, dist_grad);
    double f2 = 0.0;
    if (dist < opt->dist0) f2 = pow(dist - opt->dist0, 2);
    f_dist += f2;

    /* vel acc cost */
    static double vm = opt->max_vel, am = opt->max_acc,
                  ts = opt->bspline_interval;
    static double vm2 = vm * vm;
    static double am2 = am * am;
    static double ts_inv = 1 / ts;
    Eigen::Vector3d q32;
    Eigen::Vector3d q21;
    Eigen::Vector3d q132;
    Eigen::Vector3d q243;
    Eigen::Vector3d q021;

    /* f_exp_vel = [ (q2x - q1x)^2 - (vm*ts)^2] ^2}*/
    q21 = q2 - q1;
    double fv = 0.0;
    double conv21[3] = {0, 0, 0};
    for (int j = 0; j < 3; ++j) {
      conv21[j] = q21(j) * q21(j) * ts_inv * ts_inv - vm2;
      if (conv21[j] > 0.0) fv += pow(conv21[j], 2);
    }
    f_exp_vel += fv;

    /* f_exp_acc = [ (q3x+q1x-2 q2x)^2 - (vm*ts^2)^2] ^2}*/
    double fa = 0.0;
    double cona132[3] = {0, 0, 0};
    if (!last) {
      q132 = q1 + q3 - 2 * q2;
      for (int j = 0; j < 3; ++j) {
        cona132[j] =
            q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona132[j] > 0.0) fa += pow(cona132[j], 2);
      }
      f_exp_acc += fa;
    }

    double cona021[3] = {0, 0, 0};
    if (i == opt->order) {
      q0 = opt->control_points.row(opt->order - 2);
      q021 = q0 + q2 - 2 * q1;
      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        cona021[j] =
            q021(j) * q021(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2;
        if (cona021[j] > 0.0) fa += pow(cona021[j], 2);
      }
      f_exp_acc += fa;
    }

    /* gradient of end point cost */
    Eigen::Vector3d g_end_q1(0, 0, 0), g_end_q2(0, 0, 0), g_end_q3(0, 0, 0);
    if (last_second) {
      g_end_q1 = g_end_q3 = 2 * d_to_end * (1 / 6.0);
      g_end_q2 = 2 * d_to_end * (4 / 6.0);
    }

    /* gradient of curvature cost */
    Eigen::Vector3d g_cur_q1, g_cur_q2, g_cur_q3;
    if (!last) {
      g_cur_q1 = g_cur_q3 = 2 * (q1 + q3 - 2 * q2);
      g_cur_q2 = -4 * (q1 + q3 - 2 * q2);
    } else {
      g_cur_q1 = g_cur_q2 = g_cur_q3 = Eigen::Vector3d::Zero();
    }
    if (i == opt->order) {
      g_cur_q2 += 2 * (q0 + q2 - 2 * q1);
    }

    /* gradient of dist cost */
    Eigen::Vector3d g_dist_q2 = Eigen::Vector3d::Zero();
    if (dist < opt->dist0) g_dist_q2 = 2 * (dist - opt->dist0) * dist_grad;

    Eigen::Vector3d g_vel_q1(0, 0, 0), g_vel_q2(0, 0, 0);
    Eigen::Vector3d g_acc_q1(0, 0, 0), g_acc_q2(0, 0, 0), g_acc_q3(0, 0, 0);
    for (int j = 0; j < 3; ++j) {
      /* gradient of vel */
      if (conv21[j] > 0.0) {
        g_vel_q2(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * 2.0 * q21(j);
        g_vel_q1(j) = 2.0 * ts_inv * ts_inv * (conv21[j]) * (-2.0) * q21(j);
      }

      /* gradient of acc */
      if (!last) {
        if (cona132[j] > 0.0) {
          g_acc_q1(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                        2.0 * q132(j);
          g_acc_q2(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                        (-4.0) * q132(j);
          g_acc_q3(j) = 2.0 * ts_inv * ts_inv * ts_inv * ts_inv * (cona132[j]) *
                        2.0 * q132(j);
        }
      }

      if (i == opt->order) {
        if (cona021[j] > 0.0)
          g_acc_q2(j) += 2.0 * ts_inv * ts_inv * ts_inv * ts_inv *
                         (cona021[j]) * 2.0 * q021(j);
      }
    }

    // return gradient of function and cost
    if (i != opt->order) {
      grad[0 + 3 * (i - opt->order - 1)] +=
          opt->lamda1 * g_cur_q1(0) +
          opt->lamda3 * (g_vel_q1(0) + g_acc_q1(0)) + opt->lamda4 * g_end_q1(0);
      grad[1 + 3 * (i - opt->order - 1)] +=
          opt->lamda1 * g_cur_q1(1) +
          opt->lamda3 * (g_vel_q1(1) + g_acc_q1(1)) + opt->lamda4 * g_end_q1(1);
      grad[2 + 3 * (i - opt->order - 1)] +=
          opt->lamda1 * g_cur_q1(2) +
          opt->lamda3 * (g_vel_q1(2) + g_acc_q1(2)) + opt->lamda4 * g_end_q1(2);
    }

    grad[0 + 3 * (i - opt->order)] +=
        opt->lamda1 * g_cur_q2(0) + opt->lamda2 * g_dist_q2(0) +
        opt->lamda3 * (g_vel_q2(0) + g_acc_q2(0)) + opt->lamda4 * g_end_q2(0);
    grad[1 + 3 * (i - opt->order)] +=
        opt->lamda1 * g_cur_q2(1) + opt->lamda2 * g_dist_q2(1) +
        opt->lamda3 * (g_vel_q2(1) + g_acc_q2(1)) + opt->lamda4 * g_end_q2(1);
    grad[2 + 3 * (i - opt->order)] +=
        opt->lamda1 * g_cur_q2(2) + opt->lamda2 * g_dist_q2(2) +
        opt->lamda3 * (g_vel_q2(2) + g_acc_q2(2)) + opt->lamda4 * g_end_q2(2);

    if (!last) {
      grad[0 + 3 * (i - opt->order + 1)] += opt->lamda1 * g_cur_q3(0) +
                                            opt->lamda3 * g_acc_q3(0) +
                                            opt->lamda4 * g_end_q3(0);
      grad[1 + 3 * (i - opt->order + 1)] += opt->lamda1 * g_cur_q3(1) +
                                            opt->lamda3 * g_acc_q3(1) +
                                            opt->lamda4 * g_end_q3(1);
      grad[2 + 3 * (i - opt->order + 1)] += opt->lamda1 * g_cur_q3(2) +
                                            opt->lamda3 * g_acc_q3(2) +
                                            opt->lamda4 * g_end_q3(2);
    }
  }

  /* f = l1 * f_curvature + l2 * f_dist + l3 * (f_exp_vel + f_exp_acc)  */
  f_combine = opt->lamda1 * f_curvature + opt->lamda2 * f_dist +
              opt->lamda3 * (f_exp_vel + f_exp_acc) + opt->lamda4 * f_end;

  if (optnum % 30 == 0) {
    SETCY << optnum << " cur: " << opt->lamda1 * f_curvature
          << " , dist: " << opt->lamda2 * f_dist
          << ", exp:" << opt->lamda3 * (f_exp_acc + f_exp_vel)
          << ", total: " << f_combine << REC;
    // cout << "average curvature cost: "
    //      << opt->lamda1 * f_curvature / (opt->control_points.rows() - 6)
    //      << "\n avg dist cost: "
    //      << opt->lamda2 * f_dist / (opt->control_points.rows() - 6)
    //      << "\ncontrol points num: " << opt->control_points.rows() - 6 <<
    //      endl;
    // cout << "lamda: " << opt->lamda1 << endl;
  }

  /* save the min cost result */
  if (f_combine < opt->min_cost) {
    opt->min_cost = f_combine;
    opt->best_variable = x;
  }
  return f_combine;
}

double GradBandOptimizer::costFuncMinCurvatureExp(const std::vector<double>& x,
                                                  std::vector<double>& grad,
                                                  void* func_data) {
  GradBandOptimizer* opt = reinterpret_cast<GradBandOptimizer*>(func_data);
  grad.resize(opt->variable_num);
  for (int i = 0; i < grad.size(); ++i) {
    grad[i] = 0;
  }
  static int optnum = 0;
  ++optnum;

  /* optimize unfixed control points of bspline */
  double f_combine = 0.0, f_curvature = 0.0, f_exp_vel = 0.0, f_exp_acc = 0.0,
         f_dist = 0.0;
  for (int i = 0; i < opt->control_points.rows(); ++i) {
    if (i < opt->start_id || i >= opt->end_id) continue;

    /* get the current, back and front control_points */
    Eigen::Vector3d q1, q2, q3;
    if (i == opt->start_id)
      q1 = opt->control_points.row(opt->start_id - 1);
    else {
      q1(0) = x[0 + 3 * (i - opt->start_id - 1)];
      q1(1) = x[1 + 3 * (i - opt->start_id - 1)];
      q1(2) = x[2 + 3 * (i - opt->start_id - 1)];
    }

    q2(0) = x[0 + 3 * (i - opt->start_id)];
    q2(1) = x[1 + 3 * (i - opt->start_id)];
    q2(2) = x[2 + 3 * (i - opt->start_id)];

    if (i == opt->end_id - 1)
      q3 = opt->control_points.row(opt->end_id);
    else {
      q3(0) = x[0 + 3 * ((i - opt->start_id + 1))];
      q3(1) = x[1 + 3 * ((i - opt->start_id + 1))];
      q3(2) = x[2 + 3 * ((i - opt->start_id + 1))];
    }

    /* f_curvature = Sigma ||q1+q3-2q2||^2 */
    double f1 = (q1 + q3 - 2 * q2).squaredNorm();
    Eigen::Vector3d q0, q4;
    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
      f1 += (q0 + q2 - 2 * q1).squaredNorm();
    } else if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      f1 += (q2 + q4 - 2 * q3).squaredNorm();
    }
    f_curvature += f1;

    /* f_dist = (d-d0)^2, if d<d0;  0, if d>=d0 */
    Eigen::Vector3d dist_grad;
    double dist;
    opt->getDistanceAndGradient(q2, dist, dist_grad);
    double f2 = 0.0;
    if (dist < opt->dist0) f2 = pow(dist - opt->dist0, 2);
    f_dist += f2;

    /* only constrain the first few points */
    static double vm = opt->max_vel, am = opt->max_acc,
                  ts = opt->bspline_interval;
    static double vm2 = vm * vm;
    static double am2 = am * am;
    static double ts_inv = 1 / ts;
    const int constrained_num = 5;
    Eigen::Vector3d q32;
    Eigen::Vector3d q21;
    Eigen::Vector3d q132;
    Eigen::Vector3d q243;
    Eigen::Vector3d q021;
    // if (i < opt->order + constrained_num ||
    //     i >= opt->control_points.rows() - opt->order - constrained_num) {
    /* f_exp_vel = alpha * exp{beta * [ (q2x-q1x)^2 - (vm*ts)^2] } */

    q21 = q2 - q1;
    double fv = 0.0;
    for (int j = 0; j < 3; ++j)
      fv += opt->alpha *
            exp(opt->beta * (q21(j) * q21(j) * ts_inv * ts_inv - vm2));
    f_exp_vel += fv;

    if (i == opt->end_id - 1) {
      q32 = q3 - q2;
      fv = 0.0;
      for (int j = 0; j < 3; ++j)
        fv += opt->alpha *
              exp(opt->beta * (q32(j) * q32(j) * ts_inv * ts_inv - vm2));
      f_exp_vel += fv;
    }

    /* f_exp_acc = alpha * exp{beta * [ (q3x+q1x-2 q2x)^2 - (vm*ts^2)^2] }*/
    q132 = q1 + q3 - 2 * q2;
    double fa = 0.0;
    for (int j = 0; j < 3; ++j)
      fa += opt->alpha *
            exp(opt->beta *
                (q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2));
    f_exp_acc += fa;

    if (i == opt->start_id) {
      q0 = opt->control_points.row(opt->start_id - 2);
      q021 = q0 + q2 - 2 * q1;

      fa = 0.0;
      for (int j = 0; j < 3; ++j) {
        fa +=
            opt->alpha *
            exp(opt->beta *
                (q021(j) * q021(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2));
      }
      f_exp_acc += fa;
    }

    if (i == opt->end_id - 1) {
      q4 = opt->control_points.row(opt->end_id + 1);
      q243 = q2 + q4 - 2 * q3;
      fa = 0.0;
      for (int j = 0; j < 3; ++j)
        fa +=
            opt->alpha *
            exp(opt->beta *
                (q243(j) * q243(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2));
      f_exp_acc += fa;
    }
    // }

    /* gradient of curvature cost */
    Eigen::Vector3d g_cur_q1, g_cur_q2, g_cur_q3;
    g_cur_q1 = g_cur_q3 = 2 * (q1 + q3 - 2 * q2);
    g_cur_q2 = -4 * (q1 + q3 - 2 * q2);

    if (i == opt->start_id) {
      g_cur_q2 += 2 * (q0 + q2 - 2 * q1);
    } else if (i == opt->end_id - 1) {
      g_cur_q2 += 2 * (q2 + q4 - 2 * q3);
    }

    /* gradient of dist cost */
    Eigen::Vector3d g_dist_q2 = Eigen::Vector3d::Zero();
    if (dist < opt->dist0) g_dist_q2 = 2 * (dist - opt->dist0) * dist_grad;

    Eigen::Vector3d g_vel_q1(0, 0, 0), g_vel_q2(0, 0, 0);
    Eigen::Vector3d g_acc_q1(0, 0, 0), g_acc_q2(0, 0, 0), g_acc_q3(0, 0, 0);
    // if (i < opt->order + constrained_num ||
    //     i >= opt->control_points.rows() - opt->order - constrained_num) {
    for (int j = 0; j < 3; ++j) {
      /* gradient of exp vel */
      g_vel_q2(j) = opt->alpha * opt->beta * ts_inv * ts_inv *
                    exp(opt->beta * (q21(j) * q21(j) * ts_inv * ts_inv - vm2)) *
                    2.0 * q21(j);
      g_vel_q1(j) = opt->alpha * opt->beta * ts_inv * ts_inv *
                    exp(opt->beta * (q21(j) * q21(j) * ts_inv * ts_inv - vm2)) *
                    (-2.0) * q21(j);
      if (i == opt->end_id - 1)
        g_vel_q2(j) +=
            opt->alpha * opt->beta * ts_inv * ts_inv *
            exp(opt->beta * (q32(j) * q32(j) * ts_inv * ts_inv - vm2)) *
            (-2.0) * q32(j);

      /* gradient of exp acc */
      g_acc_q1(j) =
          opt->alpha * opt->beta * ts_inv * ts_inv * ts_inv * ts_inv *
          exp(opt->beta *
              (q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2)) *
          2.0 * q132(j);
      g_acc_q2(j) =
          opt->alpha * opt->beta * ts_inv * ts_inv * ts_inv * ts_inv *
          exp(opt->beta *
              (q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2)) *
          (-4.0) * q132(j);
      g_acc_q3(j) =
          opt->alpha * opt->beta * ts_inv * ts_inv * ts_inv * ts_inv *
          exp(opt->beta *
              (q132(j) * q132(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2)) *
          2.0 * q132(j);
      if (i == opt->start_id)
        g_acc_q2(j) +=
            opt->alpha * opt->beta * ts_inv * ts_inv * ts_inv * ts_inv *
            exp(opt->beta *
                (q021(j) * q021(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2)) *
            2.0 * q021(j);
      if (i == opt->end_id - 1)
        g_acc_q2(j) +=
            opt->alpha * opt->beta * ts_inv * ts_inv * ts_inv * ts_inv *
            exp(opt->beta *
                (q243(j) * q243(j) * ts_inv * ts_inv * ts_inv * ts_inv - am2)) *
            2.0 * q243(j);
    }
    // }

    // return gradient of function and cost
    if (i != opt->start_id) {
      grad[0 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(0) + opt->lamda3 * (g_vel_q1(0) + g_acc_q1(0));
      grad[1 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(1) + opt->lamda3 * (g_vel_q1(1) + g_acc_q1(1));
      grad[2 + 3 * (i - opt->start_id - 1)] +=
          opt->lamda1 * g_cur_q1(2) + opt->lamda3 * (g_vel_q1(2) + g_acc_q1(2));
    }

    grad[0 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(0) + opt->lamda2 * g_dist_q2(0) +
        opt->lamda3 * (g_vel_q2(0) + g_acc_q2(0));
    grad[1 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(1) + opt->lamda2 * g_dist_q2(1) +
        opt->lamda3 * (g_vel_q2(1) + g_acc_q2(1));
    grad[2 + 3 * (i - opt->start_id)] +=
        opt->lamda1 * g_cur_q2(2) + opt->lamda2 * g_dist_q2(2) +
        opt->lamda3 * (g_vel_q2(2) + g_acc_q2(2));

    if (i != opt->end_id - 1) {
      grad[0 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(0) + opt->lamda3 * g_acc_q3(0);
      grad[1 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(1) + opt->lamda3 * g_acc_q3(1);
      grad[2 + 3 * (i - opt->start_id + 1)] +=
          opt->lamda1 * g_cur_q3(2) + opt->lamda3 * g_acc_q3(2);
    }
  }

  /* f = l1 * f_curvature + l2 * f_dist + l3 * (f_exp_vel + f_exp_acc)  */
  f_combine = opt->lamda1 * f_curvature + opt->lamda2 * f_dist +
              opt->lamda3 * (f_exp_vel + f_exp_acc);

  if (optnum % 5 == 0) {
    SETCY << optnum << " cur: " << opt->lamda1 * f_curvature
          << " , dist: " << opt->lamda2 * f_dist
          << ", exp:" << opt->lamda3 * (f_exp_acc + f_exp_vel)
          << ", total: " << f_combine << REC;
    // cout << "average curvature cost: "
    //      << opt->lamda1 * f_curvature / (opt->control_points.rows() - 6)
    //      << "\n avg dist cost: "
    //      << opt->lamda2 * f_dist / (opt->control_points.rows() - 6)
    //      << "\ncontrol points num: " << opt->control_points.rows() - 6 <<
    //      endl;
    // cout << "lamda: " << opt->lamda1 << endl;
  }

  /* save the min cost result */
  if (f_combine < opt->min_cost) {
    opt->min_cost = f_combine;
    opt->best_variable = x;
  }
  return f_combine;
}

double GradBandOptimizer::velConstraint(const std::vector<double>& x,
                                        std::vector<double>& grad, void* data) {
  ConstrainData* cons = reinterpret_cast<ConstrainData*>(data);
  grad.resize(cons->variable_num);

  // get the current idx of control_points
  // int idx = cons->opt->current_optimize_id;
  // int axis = cons->opt->current_axis;
  // int sign = cons->opt->current_sign;
  int idx = cons->idx;
  int axis = cons->axis;
  int sign = cons->sign;
  // cout << "vel cons:" << endl;
  // cons->show();

  // the control control_points of velocity is (pi+1-pi)/dt, when the interal
  // is dt we try bspline_interval = 2 optimization variables, (x1,y1,z1,
  // x2,y2,z2
  // ... xn,yn,zn) notice that the first and last point is not added in the
  // optimized variables
  double mi, mi1, vi;
  for (int i = 0; i < cons->variable_num; ++i) grad[i] = 0.0;

  // constrain
  if (idx == 0) {
    mi = cons->first_pt(axis);
    mi1 = x[axis];

    grad[axis] = double(sign);
  } else if (idx == cons->point_num - 2) {
    mi = x[3 * (idx - 1) + axis];
    mi1 = cons->last_pt(axis);

    grad[3 * (idx - 1) + axis] = -double(sign);
  } else {
    mi = x[3 * (idx - 1) + axis];
    mi1 = x[3 * idx + axis];

    grad[3 * (idx - 1) + axis] = -double(sign);
    grad[3 * idx + axis] = double(sign);
  }

  // NLopt use g(x) <= 0 constraint
  static double vm = 3.5, dt = 0.1;
  return sign * (mi1 - mi) - vm * dt;
}

double GradBandOptimizer::accConstraint(const std::vector<double>& x,
                                        std::vector<double>& grad, void* data) {
  ConstrainData* cons = reinterpret_cast<ConstrainData*>(data);
  grad.resize(cons->variable_num);

  // get the current idx of control_points
  int idx = cons->idx;
  int axis = cons->axis;
  int sign = cons->sign;
  // cout << "acc cons" << endl;
  // cons->show();

  // the control control_points of acceleration is (pi+2 - 2pi+1 + pi)/dt^2,
  // when the interal is dt we try bspline_interval = 2 optimization
  // variables, (x1,y1,z1, x2,y2,z2 ... xn,yn,zn) notice that the first and
  // last point is not added in the optimized variables
  double mi, mi1, mi2, ai;
  for (int i = 0; i < cons->variable_num; ++i) grad[i] = 0.0;

  // constrain
  if (idx == 0) {
    mi = cons->first_pt(axis);
    mi1 = x[axis];
    mi2 = x[axis + 3];

    grad[axis] = -2.0 * double(sign);
    grad[axis + 3] = double(sign);
  } else if (idx == cons->point_num - 3) {
    mi = x[3 * (idx - 1) + axis];
    mi1 = x[3 * idx + axis];
    mi2 = cons->last_pt(axis);

    grad[3 * (idx - 1) + axis] = double(sign);
    grad[3 * idx + axis] = -2.0 * double(sign);
  } else {
    mi = x[3 * (idx - 1) + axis];
    mi1 = x[3 * idx + axis];
    mi2 = x[3 * (idx + 1) + axis];

    grad[3 * (idx - 1) + axis] = double(sign);
    grad[3 * idx + axis] = -2.0 * double(sign);
    grad[3 * (idx + 1) + axis] = double(sign);
  }

  // NLopt use g(x) <= 0 constraint
  static double am = 3.0, dt = 0.1;
  return sign * (mi2 - 2 * mi1 + mi) - am * dt * dt;
}
