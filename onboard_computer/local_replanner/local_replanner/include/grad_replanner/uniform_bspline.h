#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <iostream>
#include "grad_replanner/utils/font_color.h"
using namespace std;

class UniformBspline {
 private:
  /* bspline */
  int p, n, m;
  Eigen::MatrixXd control_points;
  std::vector<Eigen::MatrixXd> M;  // 012,345
  Eigen::VectorXd u;
  double interval;

  /* use initial segment or not */
  bool use_init_segment;
  Eigen::MatrixXd coef_init;
  double t_init;
  int start_num;

  Eigen::VectorXd getU(double u);
  Eigen::MatrixXd getPi(int idx);
  Eigen::MatrixXd getDerivativeControlPoints();

 public:
  UniformBspline() {}
  UniformBspline(Eigen::MatrixXd points, int order, double interval,
                 bool auto_extend = false);
  ~UniformBspline();

  void setInitSegment(Eigen::MatrixXd coef, double t_init, int start_num);

  Eigen::MatrixXd getControlPoint() { return control_points; }

  void getRegion(double& um, double& um_p);

  Eigen::Vector3d evaluate(double u);

  UniformBspline getDerivative();

  static void getControlPointEqu3(Eigen::MatrixXd samples, double ts,
                                  Eigen::MatrixXd& control_pts);

  static void checkAndInsertControlPoints(Eigen::MatrixXd& control_pts,
                                          Eigen::VectorXd& weight, int order,
                                          double ts, double limit_vel,
                                          double limit_acc);
  static bool inVector(vector<int> vec, int id);

  /* check feasibility and recompute initial segment */
  static bool checkFeasibility(const Eigen::MatrixXd& ctrl_pts,
                               Eigen::VectorXi& flag, double& ts_new, double ts,
                               double limit_vel, double limit_acc);

  static bool computeInitSegment(const Eigen::MatrixXd& ctrl_pts,
                                 int& start_num, const double ts,
                                 const double max_vel, const double max_acc,
                                 const Eigen::Vector3d& init_pt,
                                 Eigen::Vector3d& init_vel,
                                 Eigen::MatrixXd& coef, double& t_init);

  static bool computeInitSegmentMinJerk(const Eigen::MatrixXd& ctrl_pts,
                                        int& start_num, const double ts,
                                        const double max_vel,
                                        const double max_acc,
                                        const Eigen::Vector3d& init_pt,
                                        Eigen::Vector3d& init_vel,
                                        Eigen::MatrixXd& coef, double& t_init);
  static vector<double> quadratic(double a, double b, double c);
  static std::vector<double> cubic(double a, double b, double c, double d);
  static std::vector<double> quartic(double a, double b, double c, double d,
                                     double e);

  static void reallocateCtrlPts(Eigen::MatrixXd& ctrl_pts, double ts, int order,
                                double limit_vel, double limit_acc);
};

// control points is a (n+1)x3 matrix
UniformBspline::UniformBspline(Eigen::MatrixXd points, int order,
                               double interval, bool auto_extend) {
  this->p = order;
  if (auto_extend) {
    control_points = Eigen::MatrixXd::Zero(points.rows() + 2 * this->p, 3);
    for (int i = 0; i < this->p; ++i) {
      control_points.row(i) = points.row(0);
      control_points.row(control_points.rows() - 1 - i) =
          points.row(points.rows() - 1);
    }
    control_points.block(this->p, 0, points.rows(), 3) = points;
    this->n = points.rows() + 2 * this->p - 1;
  } else {
    control_points = points;
    this->n = points.rows() - 1;
  }

  this->m = this->n + this->p + 1;

  // calculate knots vector
  this->interval = interval;
  this->u = Eigen::VectorXd::Zero(this->m + 1);
  for (int i = 0; i <= this->m; ++i) {
    if (i <= this->p)
      this->u(i) = double(-this->p + i) * this->interval;

    else if (i > this->p && i <= this->m - this->p) {
      this->u(i) = this->u(i - 1) + this->interval;
    } else if (i > this->m - this->p) {
      this->u(i) = this->u(i - 1) + this->interval;
    }
  }

  // initialize the M3-6 matrix
  this->M.resize(5);
  Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(2, 2);
  Eigen::MatrixXd M3 = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd M4 = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd M5 = Eigen::MatrixXd::Zero(5, 5);
  Eigen::MatrixXd M6 = Eigen::MatrixXd::Zero(6, 6);

  M2 << 1.0, 0.0, -1.0, 1.0;
  M3 << 1.0, 1.0, 0.0, -2.0, 2.0, 0.0, 1.0, -2.0, 1.0;
  M4 << 1.0, 4.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, 3.0, -6.0, 3.0, 0.0, -1.0, 3.0,
      -3.0, 1.0;
  M5 << 1.0, 11.0, 11.0, 1.0, 0, -4.0, -12.0, 12.0, 4.0, 0, 6.0, -6.0, -6.0,
      6.0, 0, -4.0, 12.0, -12.0, 4.0, 0, 1.0, -4.0, 6.0, -4.0, 1.0;
  M6 << 1, 26, 66, 26, 1, 0, -5, -50, 0, 50, 5, 0, 10, 20, -60, 20, 10, 0, -10,
      20, 0, -20, 10, 0, 5, -20, 30, -20, 5, 0, -1, 5, -10, 10, -5, 1;
  M3 /= 2.0;
  M4 /= 3.0 * 2;
  M5 /= 4.0 * 3.0 * 2.0;
  M6 /= 5.0 * 4.0 * 3.0 * 2.0;

  M[0] = M2;
  M[1] = M3;
  M[2] = M4;
  M[3] = M5;
  M[4] = M6;

  use_init_segment = false;

  // show the result
  // cout << "p: " << p << "  n: " << n << "  m: " << m << endl;
  // cout << "control pts:\n" << control_points << "\nknots:\n" <<
  // this->u.transpose() << endl; cout << "M3:\n" << M[0] << "\nM4:\n" << M[1]
  // << "\nM5:\n" << M[2] << endl;
}

UniformBspline::~UniformBspline() {}

void UniformBspline::setInitSegment(Eigen::MatrixXd coef, double t_init,
                                    int start_num) {
  this->use_init_segment = true;
  this->coef_init = coef;
  this->t_init = t_init;
  this->start_num = start_num;
}

void UniformBspline::getRegion(double& um, double& um_p) {
  um = this->u(this->p);
  if (!use_init_segment)
    um_p = this->u(this->m - this->p);
  else
    um_p = this->u(this->m - this->p) - interval * (start_num + 1) + t_init;
}

Eigen::VectorXd UniformBspline::getU(double u) {
  Eigen::VectorXd uv = Eigen::VectorXd::Zero(this->p + 1);
  uv(0) = 1.0;
  for (int i = 1; i <= this->p; ++i) {
    uv(i) = uv(i - 1) * u;
  }
  return uv;
}

Eigen::MatrixXd UniformBspline::getPi(int idx) {
  Eigen::MatrixXd pi = control_points.block(idx - p, 0, p + 1, 3);
  return pi;
}

Eigen::Vector3d UniformBspline::evaluate(double u) {
  // if (u < this->u(this->p) || u > this->u(this->m - this->p))
  // return Eigen::Vector3d::Zero(3);
  // determine which [ui,ui+1] lay in
  if (use_init_segment && u <= t_init) {
    Eigen::VectorXd px, py, pz;
    px = coef_init.col(0);
    py = coef_init.col(1);
    pz = coef_init.col(2);

    Eigen::VectorXd t_vector(6);
    t_vector << pow(u, 5), pow(u, 4), pow(u, 3), pow(u, 2), u, 1;

    Eigen::Vector3d val;
    val(0) = t_vector.dot(px);
    val(1) = t_vector.dot(py);
    val(2) = t_vector.dot(pz);

    return val;
  } else if (use_init_segment && u > t_init) {
    u = u - t_init + interval * (start_num + 1);
  }

  int idx = this->p;
  while (true) {
    if (this->u(idx + 1) >= u) break;
    ++idx;
  }

  u = (u - this->u(idx)) / (this->u(idx + 1) - this->u(idx));

  // get u vector and pi-p -> pi
  Eigen::VectorXd uv = this->getU(u);
  Eigen::MatrixXd pi = this->getPi(idx);
  // cout << "uv:" << uv.transpose() << "\npi: " << pi.transpose() << endl;

  // use p = u'*M*pi
  Eigen::Vector3d val = (uv.transpose() * M[p - 1] * pi).transpose();
  return val;
}

Eigen::MatrixXd UniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p-1
  // control point Qi = p*(Pi+1-Pi)/(ui+p+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points.rows() - 1, 3);
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) = p * (control_points.row(i + 1) - control_points.row(i)) /
                 (u(i + p + 1) - u(i + 1));
  }
  return ctp;
}

UniformBspline UniformBspline::getDerivative() {
  Eigen::MatrixXd ctp = this->getDerivativeControlPoints();
  UniformBspline derivative = UniformBspline(ctp, p - 1, this->interval, false);

  if (use_init_segment) {
    Eigen::MatrixXd coef_transform(6, 6);
    coef_transform.setZero();
    for (int i = 0; i < 5; ++i) {
      coef_transform(i + 1, i) = 5 - i;
    }

    derivative.setInitSegment(coef_transform * coef_init, this->t_init,
                              this->start_num);
  }
  return derivative;
}

// input :
//      sample : 3 x (K+2) (for 3 order) for x, y, z sample
//      ts
// output:
//      control_pts (K+6)x3
void UniformBspline::getControlPointEqu3(Eigen::MatrixXd samples, double ts,
                                         Eigen::MatrixXd& control_pts) {
  int K = samples.cols() - 4;

  // write A
  Eigen::VectorXd prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 4);

  for (int i = 0; i < K + 2; ++i) A.block(i, i, 1, 3) = prow.transpose();

  A.block(K + 2, 0, 1, 3) = A.block(K + 3, K + 1, 1, 3) = vrow.transpose();

  // cout << "A:\n" << A << endl;
  A.block(0, 0, K + 2, K + 4) = (1 / 6.0) * A.block(0, 0, K + 2, K + 4);
  A.block(K + 2, 0, 2, K + 4) = (1 / 2.0 / ts) * A.block(K + 2, 0, 2, K + 4);

  // write b
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K + 4; ++i) {
    bx(i) = samples(0, i);
    by(i) = samples(1, i);
    bz(i) = samples(2, i);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  control_pts.resize(K + 4, 3);
  control_pts.col(0) = px;
  control_pts.col(1) = py;
  control_pts.col(2) = pz;
}

/* recompute initial segment of the trajectory by optimal control */
bool UniformBspline::computeInitSegmentMinJerk(
    const Eigen::MatrixXd& ctrl_pts, int& start_num, const double ts,
    const double max_vel, const double max_acc, const Eigen::Vector3d& init_pt,
    Eigen::Vector3d& init_vel, Eigen::MatrixXd& coef, double& t_init) {
  const int try_num = 10;
  for (start_num = 2; start_num < try_num; ++start_num) {
    /* solve optimal T candidates */
    Eigen::Vector3d x1, v1, a1, c1, c2, c3, x0, v0, a0, dv;
    c1 = ctrl_pts.row(1 + start_num);
    c2 = ctrl_pts.row(2 + start_num);
    c3 = ctrl_pts.row(3 + start_num);

    x0 = (1 / 6.0) * (c1 + 4 * c2 + c3);  // start and end inverse here
    v0 = -(1 / 2.0 / ts) * (c3 - c1);
    a0 = (c1 + c3 - 2 * c2) / ts / ts;
    x1 = init_pt;
    v1 = -init_vel;

    double d1 = -16.0 * a0.dot(a0);
    double d2 = -168.0 * a0.dot(v0) - 72.0 * a0.dot(v1);
    double d3 = -320.0 * a0.dot(x0) + 320.0 * a0.dot(x1) - 512.0 * v0.dot(v0) -
                576.0 * v0.dot(v1) - 192.0 * v1.dot(v1);
    double d4 = -2000.0 * v0.dot(x0) + 2000.0 * v0.dot(x1) -
                1200.0 * v1.dot(x0) + 1200.0 * v1.dot(x1);
    double d5 =
        -1920.0 * x0.dot(x0) + 3840.0 * x0.dot(x1) - 1920.0 * x1.dot(x1);
    vector<double> roots = quartic(d1, d2, d3, d4, d5);

    /* choose the lowest cost T*/
    Eigen::Vector3d dp = x1 - x0;
    double mx = max(fabs(dp(0)), fabs(dp(1)));
    mx = max(mx, fabs(dp(2)));
    double t_bar = mx / max_vel;
    double op_t = t_bar;
    double cost = 1000000000;
    for (auto t : roots) {
      if (t < t_bar) continue;
      if (t > 4 * (start_num + 1) * ts) continue;

      double c =
          1.0 *
          (8.0 * pow(t, 4) * a0.dot(a0) + 56.0 * pow(t, 3) * a0.dot(v0) +
           24.0 * pow(t, 3) * a0.dot(v1) + 80.0 * pow(t, 2) * a0.dot(x0) -
           80.0 * pow(t, 2) * a0.dot(x1) + 128.0 * pow(t, 2) * v0.dot(v0) +
           144.0 * pow(t, 2) * v0.dot(v1) + 48.0 * pow(t, 2) * v1.dot(v1) +
           400.0 * t * v0.dot(x0) - 400.0 * t * v0.dot(x1) +
           240.0 * t * v1.dot(x0) - 240.0 * t * v1.dot(x1) +
           320.0 * x0.dot(x0) - 640.0 * x0.dot(x1) + 320.0 * x1.dot(x1)) /
          pow(t, 6);
      if (c < cost) {
        cost = c;
        op_t = t;
      }
    }
    t_init = op_t;
    cout << "opt t: " << t_init << endl;

    /* compute coefficient */
    Eigen::MatrixXd A(3, 2), B(2, 3);
    A << 320.0, -120 * t_init, -200 * t_init, 72 * t_init * t_init,
        40 * t_init * t_init, -12 * t_init * t_init * t_init;
    A *= (1 / pow(t_init, 5));

    B.row(0) = (x1 - x0 - v0 * t_init - 0.5 * a0 * t_init * t_init).transpose();
    B.row(1) = (v1 - v0 - a0 * t_init).transpose();

    Eigen::MatrixXd coef_inv(6, 3);
    coef_inv.block(0, 0, 3, 3) = A * B;
    coef_inv.row(0) *= 1 / 120.0;
    coef_inv.row(1) *= 1 / 24.0;
    coef_inv.row(2) *= 1 / 6.0;
    coef_inv.row(3) = 0.5 * a0.transpose();
    coef_inv.row(4) = v0.transpose();
    coef_inv.row(5) = x0.transpose();

    /* inverse the coef */
    coef.resize(6, 3);
    for (int i = 0; i < 3; ++i) {
      Eigen::VectorXd coefp = coef_inv.col(i);
      coef(0, i) = -coef_inv(0, i);
      coef(1, i) = 5 * t_init * coef_inv(0, i) + coef_inv(1, i);
      coef(2, i) = -10 * pow(t_init, 2) * coef_inv(0, i) -
                   4 * t_init * coef_inv(1, i) - coef_inv(2, i);
      coef(3, i) = 10 * pow(t_init, 3) * coef_inv(0, i) +
                   6 * pow(t_init, 2) * coef_inv(1, i) +
                   3 * t_init * coef_inv(2, i) + coef_inv(3, i);
      coef(4, i) = -5 * pow(t_init, 4) * coef_inv(0, i) -
                   4 * pow(t_init, 3) * coef_inv(1, i) -
                   3 * pow(t_init, 2) * coef_inv(2, i) -
                   2 * t_init * coef_inv(3, i) - coef_inv(4, i);
      coef(5, i) =
          pow(t_init, 5) * coef_inv(0, i) + pow(t_init, 4) * coef_inv(1, i) +
          pow(t_init, 3) * coef_inv(2, i) + pow(t_init, 2) * coef_inv(3, i) +
          t_init * coef_inv(4, i) + coef_inv(5, i);
    }

    /* check feasibility */
    Eigen::MatrixXd coef_transform(6, 6);
    coef_transform.setZero();
    for (int j = 0; j < 5; ++j) {
      coef_transform(j + 1, j) = 5 - j;
    }
    bool feasible = true;
    for (int i = 0; i < 3; ++i) {
      Eigen::VectorXd coefp = coef.col(i), coefv, coefa, coefj, t_vector(6);
      coefv = coef_transform * coefp;
      coefa = coef_transform * coefv;
      coefj = coef_transform * coefa;

      /* check vel extreme */
      vector<double> ext_v = cubic(coefa(2), coefa(3), coefa(4), coefa(5));
      ext_v.push_back(1e-4);
      ext_v.push_back(t_init - 1e-4);

      for (auto t_ex : ext_v) {
        if (t_ex < 0.0 || t_ex > t_init) continue;

        for (int j = 0; j < 6; ++j) t_vector(j) = pow(t_ex, 5 - j);
        double vel = t_vector.dot(coefv);

        if (fabs(vel) > max_vel) {
          feasible = false;
          break;
        }
      }
      if (!feasible) break;

      /* check acc extreme */
      vector<double> ext_a = quadratic(coefj(3), coefj(4), coefj(5));
      ext_a.push_back(1e-4);
      ext_a.push_back(t_init - 1e-4);

      for (auto t_ex : ext_a) {
        if (t_ex < 0.0 || t_ex > t_init) continue;

        for (int j = 0; j < 6; ++j) t_vector(j) = pow(t_ex, 5 - j);
        double acc = t_vector.dot(coefa);

        if (fabs(acc) > max_acc) {
          feasible = false;
          break;
        }
      }
      if (!feasible) break;
    }
    if (feasible) {
      SETY << "[BSpline]: replace " << start_num + 1
           << " segment, init time: " << t_init << REC;
      return true;
    } else {
      SETY << "[BSpline]: no feasible init trajectory in " << start_num + 1
           << " try." << REC;
      if (start_num == try_num - 1) return false;
    }
  }
}

/* recompute initial segment of the trajectory, init acc is determined by a
 * heuristic */
bool UniformBspline::computeInitSegment(
    const Eigen::MatrixXd& ctrl_pts, int& start_num, const double ts,
    const double max_vel, const double max_acc, const Eigen::Vector3d& init_pt,
    Eigen::Vector3d& init_vel, Eigen::MatrixXd& coef, double& t_init) {
  const int try_num = 10;
  for (start_num = 6; start_num < try_num; ++start_num) {
    /* get the semgnet end pos, vel, acc */
    Eigen::Vector3d x1, v1, a1, c1, c2, c3, x0, v0, a0, dv;
    c1 = ctrl_pts.row(1 + start_num);
    c2 = ctrl_pts.row(2 + start_num);
    c3 = ctrl_pts.row(3 + start_num);

    double predict_time = (start_num + 1) * ts;
    a0 = dv / predict_time;

    Eigen::Vector3d c01, c02, c03;
    c01 = ctrl_pts.row(0);
    c02 = ctrl_pts.row(1);
    c03 = ctrl_pts.row(2);

    // x0 = (1 / 6.0) * (c01 + 4 * c02 + c03);
    // v0 = (1 / 2.0 / ts) * (c03 - c01);
    // a0 = (c01 + c03 - 2 * c02) / ts / ts;
    x0 = init_pt;
    v0 = init_vel;
    // a0 = init_acc;

    x1 = (1 / 6.0) * (c1 + 4 * c2 + c3);
    v1 = (1 / 2.0 / ts) * (c3 - c1);
    a1 = (c1 + c3 - 2 * c2) / ts / ts;

    /* solve for the optimal time, assume zero initial acc */
    double d1, d2, d3, d4, d5;
    d1 = 0.085714285714289 * a0.dot(a0) + 0.0285714285714107 * a0.dot(a1) +
         0.085714285714289 * a1.dot(a1);
    d2 = 0.0;
    d3 = -0.857142857142662 * a0.dot(x0) + 0.857142857142662 * a0.dot(x1) +
         0.85714285714289 * a1.dot(x0) - 0.85714285714289 * a1.dot(x1) -
         5.48571428571449 * v0.dot(v0) - 6.17142857142881 * v0.dot(v1) -
         5.48571428571404 * v1.dot(v1);
    d4 = -34.2857142857138 * v0.dot(x0) + 34.2857142857138 * v0.dot(x1) -
         34.2857142857138 * v1.dot(x0) + 34.2857142857138 * v1.dot(x1);
    d5 = -51.4285714285706 * x0.dot(x0) + 102.857142857141 * x0.dot(x1) -
         51.4285714285706 * x1.dot(x1);
    vector<double> roots = quartic(d1, d2, d3, d4, d5);

    /* choose the best time */
    Eigen::Vector3d dp = x1 - x0;
    double mx = max(fabs(dp(0)), fabs(dp(1)));
    mx = max(mx, fabs(dp(2)));
    double t_bar = mx / max_vel;
    double op_t = t_bar;
    double cost = 1000000000;
    for (auto t : roots) {
      if (t < t_bar) continue;
      if (t > 4 * (start_num + 1) * ts) continue;
      /* cal acc integral cost */
      double c =
          0.085714285714289 * t * a0.dot(a0) +
          0.0285714285714107 * t * a0.dot(a1) +
          0.085714285714289 * t * a1.dot(a1) + 0.628571428571377 * a0.dot(v0) +
          0.228571428571513 * a0.dot(v1) - 0.228571428571513 * a1.dot(v0) -
          0.62857142857149 * a1.dot(v1) + 0.857142857142662 * a0.dot(x0) / t -
          0.857142857142662 * a0.dot(x1) / t -
          0.85714285714289 * a1.dot(x0) / t +
          0.85714285714289 * a1.dot(x1) / t +
          5.48571428571449 * v0.dot(v0) / t +
          6.17142857142881 * v0.dot(v1) / t +
          5.48571428571404 * v1.dot(v1) / t +
          17.1428571428569 * v0.dot(x0) / pow(t, 2) -
          17.1428571428569 * v0.dot(x1) / pow(t, 2) +
          17.1428571428569 * v1.dot(x0) / pow(t, 2) -
          17.1428571428569 * v1.dot(x1) / pow(t, 2) +
          17.1428571428569 * x0.dot(x0) / pow(t, 3) -
          34.2857142857138 * x0.dot(x1) / pow(t, 3) +
          17.1428571428569 * x1.dot(x1) / pow(t, 3);
      if (c < cost) {
        cost = c;
        op_t = t;
      }
    }
    t_init = op_t;

    // cout << "optimal time: " << t_init << endl;

    /* solve coefficient using the optimal time */
    Eigen::Matrix3d A, B;
    Eigen::Vector3d bx, by, bz;
    A << pow(t_init, 5), pow(t_init, 4), pow(t_init, 3), 5 * pow(t_init, 4),
        4 * pow(t_init, 3), 3 * pow(t_init, 2), 20 * pow(t_init, 3),
        12 * pow(t_init, 2), 6 * t_init;
    B.row(0) = (x1 - x0 - v0 * t_init - 0.5 * a0 * t_init * t_init).transpose();
    B.row(1) = (v1 - v0 - a0 * t_init).transpose();
    B.row(2) = (a1 - a0).transpose();

    bx = B.col(0);
    by = B.col(1);
    bz = B.col(2);

    Eigen::Vector3d px = A.colPivHouseholderQr().solve(bx);
    Eigen::Vector3d py = A.colPivHouseholderQr().solve(by);
    Eigen::Vector3d pz = A.colPivHouseholderQr().solve(bz);

    /* get the coefficient */
    coef.resize(6, 3);
    coef.block(0, 0, 3, 1) = px;
    coef.block(0, 1, 3, 1) = py;
    coef.block(0, 2, 3, 1) = pz;
    coef.row(3) = 0.5 * a0.transpose();
    coef.row(4) = v0.transpose();
    coef.row(5) = x0.transpose();

    /* check feasibility */
    Eigen::MatrixXd coef_transform(6, 6);
    coef_transform.setZero();
    for (int j = 0; j < 5; ++j) {
      coef_transform(j + 1, j) = 5 - j;
    }
    bool feasible = true;
    for (int i = 0; i < 3; ++i) {
      Eigen::VectorXd coefp = coef.col(i), coefv, coefa, coefj, t_vector(6);
      coefv = coef_transform * coefp;
      coefa = coef_transform * coefv;
      coefj = coef_transform * coefa;

      /* check vel extreme */
      vector<double> ext_v = cubic(coefa(2), coefa(3), coefa(4), coefa(5));
      ext_v.push_back(1e-4);
      ext_v.push_back(t_init - 1e-4);

      for (auto t_ex : ext_v) {
        if (t_ex < 0.0 || t_ex > t_init) continue;

        for (int j = 0; j < 6; ++j) t_vector(j) = pow(t_ex, 5 - j);
        double vel = t_vector.dot(coefv);

        if (fabs(vel) > max_vel) {
          feasible = false;
          break;
        }
      }
      if (!feasible) break;

      /* check acc extreme */
      vector<double> ext_a = quadratic(coefj(3), coefj(4), coefj(5));
      ext_a.push_back(1e-4);
      ext_a.push_back(t_init - 1e-4);

      for (auto t_ex : ext_a) {
        if (t_ex < 0.0 || t_ex > t_init) continue;

        for (int j = 0; j < 6; ++j) t_vector(j) = pow(t_ex, 5 - j);
        double acc = t_vector.dot(coefa);

        if (fabs(acc) > max_acc) {
          feasible = false;
          break;
        }
      }
      if (!feasible) break;
    }
    if (feasible) {
      SETY << "[BSpline]: replace " << start_num + 1
           << " segment, init time: " << t_init << REC;
      return true;
    } else {
      SETY << "[BSpline]: no feasible init trajectory in " << start_num + 1
           << " try." << REC;
      if (start_num == try_num - 1) return false;
    }
  }
}

vector<double> UniformBspline::quadratic(double a, double b, double c) {
  vector<double> roots;

  /* delta */
  double delta = b * b - 4 * a * c;

  if (delta < 0)
    ;
  else if (delta == 0) {
    roots.push_back(-b / (2 * a));
  } else if (delta > 0) {
    roots.push_back((-b + sqrt(delta)) / (2 * a));
    roots.push_back((-b - sqrt(delta)) / (2 * a));
  }
  return roots;
}

vector<double> UniformBspline::cubic(double a, double b, double c, double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> UniformBspline::quartic(double a, double b, double c, double d,
                                       double e) {
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys =
      cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

bool UniformBspline::checkFeasibility(const Eigen::MatrixXd& ctrl_pts,
                                      Eigen::VectorXi& flag, double& ts_new,
                                      double ts, double limit_vel,
                                      double limit_acc) {
  flag = Eigen::VectorXi::Ones(ctrl_pts.rows());

  bool fea = true;
  SETY << "[Bspline]: total points size: " << ctrl_pts.rows() << REC;
  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 3; i < ctrl_pts.rows() - 2; ++i) {
    Eigen::Vector3d vel = (ctrl_pts.row(i) - ctrl_pts.row(i - 1)) / ts;
    if (fabs(vel(0)) > limit_vel || fabs(vel(1)) > limit_vel ||
        fabs(vel(2)) > limit_vel) {
      /* insert mid point */
      // SETR << "[Bspline]: Infeasible vel " << i << " :" << vel.transpose()
      //      << REC;
      fea = false;
      max_vel = max(max_vel, fabs(vel(0)));
      max_vel = max(max_vel, fabs(vel(1)));
      max_vel = max(max_vel, fabs(vel(2)));

      flag(i) = 2;
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 2; i < ctrl_pts.rows() - 2; ++i) {
    Eigen::Vector3d acc =
        (ctrl_pts.row(i + 1) + ctrl_pts.row(i - 1) - 2 * ctrl_pts.row(i)) /
        (ts * ts);
    if (fabs(acc(0)) > limit_acc || fabs(acc(1)) > limit_acc ||
        fabs(acc(2)) > limit_acc) {
      /* insert mid point */
      // SETR << "[Bspline]: Infeasible acc " << i << " :" << acc.transpose()
      //      << REC;
      fea = false;
      max_acc = max(max_acc, fabs(acc(0)));
      max_acc = max(max_acc, fabs(acc(1)));
      max_acc = max(max_acc, fabs(acc(2)));
      flag(i) = 3;
    }
  }

  cout << "max vel:" << max_vel << ", max acc:" << max_acc << endl;
  double ratio = max(max_vel / limit_vel, sqrt(max_acc / limit_acc));
  cout << "ratio:" << ratio << endl;

  if (!fea)
    ts_new = ts * ratio;
  else
    ts_new = ts;

  return fea;
}

void UniformBspline::reallocateCtrlPts(Eigen::MatrixXd& ctrl_pts, double ts,
                                       int order, double limit_vel,
                                       double limit_acc) {
  /* create the b spline */
  UniformBspline spline(ctrl_pts, order, ts);
  double us, ug;
  spline.getRegion(us, ug);

  /* forward searching along the bspline */
  vector<Eigen::Vector3d> ctrl_pts_new;
  Eigen::Vector3d p1, p2, p3, p_last, vel, acc, p_end;
  bool infeasible_vel, infeasible_acc, inrange = false;
  p1 = ctrl_pts.row(1);
  p2 = ctrl_pts.row(2);
  p_end = ctrl_pts.row(ctrl_pts.rows() - order);

  ctrl_pts_new.push_back(ctrl_pts.row(0));
  ctrl_pts_new.push_back(ctrl_pts.row(1));
  ctrl_pts_new.push_back(ctrl_pts.row(2));
  cout << "initial size: " << ctrl_pts_new.size() << endl;
  cout << "initial size: " << ctrl_pts.rows() << endl;

  vector<Eigen::Vector3d> pts_fea;
  for (double t = us; t <= ug; t += 0.001) {
    p3 = spline.evaluate(t);
    // cout << "\np3: " << p3.transpose() << endl;
    if (!inrange) {
      vel = (p3 - p2) / ts;
      infeasible_vel = fabs(vel(0)) > limit_vel || fabs(vel(1)) > limit_vel ||
                       fabs(vel(2)) > limit_vel;
      if (infeasible_vel) continue;
      // cout << "not in range, vel ok" << endl;

      acc = (p3 + p1 - 2 * p2) / (ts * ts);
      infeasible_acc = fabs(acc(0)) > limit_acc || fabs(acc(1)) > limit_acc ||
                       fabs(acc(2)) > limit_acc;
      if (infeasible_acc) continue;
      // SETR << "not in range, acc ok" << REC;

      inrange = true;
      p_last = p3;
      pts_fea.push_back(p_last);

    } else {
      vel = (p3 - p2) / ts;
      acc = (p3 + p1 - 2 * p2) / (ts * ts);

      infeasible_vel = fabs(vel(0)) > limit_vel || fabs(vel(1)) > limit_vel ||
                       fabs(vel(2)) > limit_vel;
      infeasible_acc = fabs(acc(0)) > limit_acc || fabs(acc(1)) > limit_acc ||
                       fabs(acc(2)) > limit_acc;

      if (!infeasible_vel && !infeasible_acc) {
        p_last = p3;
        pts_fea.push_back(p_last);
        // cout << "in range, keep searching" << endl;
      } else {
        // SETR << "out range, add new point" << REC;
        /* select the min acc one to be new points */
        double min_cost = 100000.0;
        Eigen::Vector3d pt_best;
        for (int i = 0; i < pts_fea.size(); ++i) {
          Eigen::Vector3d pt_temp = pts_fea[i];
          double cost = (pt_temp + p1 - 2 * p2).norm();
          if (cost < min_cost) {
            pt_best = pt_temp;
            min_cost = cost;
          }
        }

        inrange = false;
        p1 = p2;
        p2 = pt_best;
        ctrl_pts_new.push_back(pt_best);

        /* near end, stop */
        if ((pt_best - p_end).norm() < 0.3) {
          cout << "reach end" << endl;
          break;
        }
      }
    }
  }
  ctrl_pts.resize(ctrl_pts_new.size(), 3);
  for (int i = 0; i < ctrl_pts_new.size(); ++i) {
    ctrl_pts.row(i) = ctrl_pts_new[i];
  }
  cout << "new size: " << ctrl_pts_new.size() << endl;
  return;
}

bool UniformBspline::inVector(vector<int> vec, int id) {
  bool in_vector = false;

  for (int i = 0; i < vec.size(); ++i) {
    if (abs(vec[i] - id) < 10) {
      in_vector = true;
      break;
    }
  }
  return in_vector;
}

void UniformBspline::checkAndInsertControlPoints(Eigen::MatrixXd& control_pts,
                                                 Eigen::VectorXd& weight,
                                                 int order, double ts,
                                                 double limit_vel,
                                                 double limit_acc) {
  vector<Eigen::Vector3d> ctrl_pts;
  for (int i = 0; i < control_pts.rows(); ++i) {
    ctrl_pts.push_back(control_pts.row(i));
  }

  vector<double> weight_vec;
  for (int i = 0; i < weight.rows(); ++i) {
    weight_vec.push_back(weight(i));
  }

  /* record added points in vel */
  vector<int> vel_id;

  const int skip = 8;
  int start_insert = 0;
  int end_insert = 0;
  const double ratio = 1.0;
  /* check vel and insert */
  for (int i = order; i <= ctrl_pts.size() - order; ++i) {
    Eigen::Vector3d q2 = ctrl_pts[i];
    Eigen::Vector3d q1 = ctrl_pts[i - 1];
    Eigen::Vector3d vel = (q2 - q1) / ts;
    if (fabs(vel(0)) > ratio * limit_vel || fabs(vel(1)) > ratio * limit_vel ||
        fabs(vel(2)) > ratio * limit_vel) {
      Eigen::Vector3d q0, q3, q01, q12, q23, qm;
      q0 = ctrl_pts[i - 2];
      q3 = ctrl_pts[i + 1];
      q01 = q1 + (q1 - q0);
      q12 = 0.5 * (q1 + q2);
      q23 = q2 + (q2 - q3);
      qm = (1 / 6.0) * q01 + (2 / 3.0) * q12 + (1 / 6.0) * q23;
      ctrl_pts.insert(ctrl_pts.begin() + i, qm);

      /* change dist cost weight */
      weight_vec.insert(weight_vec.begin() + i, 0);
      double avg = (weight_vec[i - 1] + weight_vec[i + 1]) / 3.0;
      weight_vec[i - 1] = avg;
      weight_vec[i] = avg;
      weight_vec[i + 1] = avg;

      vel_id.push_back(i);
      i += skip;
      start_insert = skip;
    }
  }

  /* start acc */
  if (!inVector(vel_id, 2)) {
    Eigen::Vector3d q1, q2, q3, q4, acc, q12, q34, q23, qm;
    q1 = ctrl_pts[1];
    q2 = ctrl_pts[2];
    q3 = ctrl_pts[3];
    acc = (q1 + q3 - 2 * q2) / ts / ts;
    if (acc(0) > ratio * limit_acc || acc(1) > ratio * limit_acc ||
        acc(2) > ratio * limit_acc) {
      q4 = ctrl_pts[4];
      q12 = q2 + (q2 - q1);
      q23 = 0.5 * (q2 + q3);
      q34 = q3 + (q3 - q4);
      qm = (1 / 6.0) * q12 + (2 / 3.0) * q23 + (1 / 6.0) * q34;
      ctrl_pts.insert(ctrl_pts.begin() + 3, qm);
      weight_vec.insert(weight_vec.begin() + 3, 0);
      double avg = (weight_vec[2] + weight_vec[4]) / 3.0;
      weight_vec[2] = avg;
      weight_vec[3] = avg;
      weight_vec[4] = avg;
    }
  }
  /* check acc and replace, NOT include start and end acc */
  for (int i = order + start_insert; i < ctrl_pts.size() - order; ++i) {
    /* skip point near vel insertion */
    if (inVector(vel_id, i)) continue;

    Eigen::Vector3d q1, q2, q3, acc;
    q1 = ctrl_pts[i - 1];
    q2 = ctrl_pts[i];
    q3 = ctrl_pts[i + 1];
    acc = (q1 + q3 - 2 * q2) / ts / ts;
    if (acc(0) > ratio * limit_acc || acc(1) > ratio * limit_acc ||
        acc(2) > ratio * limit_acc) {
      Eigen::Vector3d q4, q12, q23, q34, qm;
      q4 = ctrl_pts[i + 2];
      q12 = q2 + (q2 - q1);
      q23 = 0.5 * (q2 + q3);
      q34 = q3 + (q3 - q4);
      qm = (1 / 6.0) * q12 + (2 / 3.0) * q23 + (1 / 6.0) * q34;
      ctrl_pts.insert(ctrl_pts.begin() + i + 1, qm);
      weight_vec.insert(weight_vec.begin() + i + 1, 0);
      double avg = (weight_vec[i] + weight_vec[i + 2]) / 3.0;
      weight_vec[i] = avg;
      weight_vec[i + 1] = avg;
      weight_vec[i + 2] = avg;
      i += skip;
      end_insert = i;
    } else if (acc(0) < -ratio * limit_acc || acc(1) < -ratio * limit_acc ||
               acc(2) < -ratio * limit_acc) {
      Eigen::Vector3d q0, q01, q12, q23, qm;
      q0 = ctrl_pts[i - 2];
      q01 = q1 + (q1 - q0);
      q12 = 0.5 * (q1 + q2);
      q23 = q2 + (q2 - q3);
      qm = (1 / 6.0) * q01 + (2 / 3.0) * q12 + (1 / 6.0) * q23;
      ctrl_pts.insert(ctrl_pts.begin() + i, qm);
      weight_vec.insert(weight_vec.begin() + i, 0);
      double avg = (weight_vec[i - 1] + weight_vec[i + 1]) / 3.0;
      weight_vec[i - 1] = avg;
      weight_vec[i] = avg;
      weight_vec[i + 1] = avg;
      i += skip;
      end_insert = i;
    }
  }
  /* end */
  int N = ctrl_pts.size();
  if (end_insert <= N - 3 && !inVector(vel_id, N - 3)) {
    Eigen::Vector3d q0, q1, q2, q3, q01, q12, q23, qm, acc;
    q0 = ctrl_pts[N - 5];
    q1 = ctrl_pts[N - 4];
    q2 = ctrl_pts[N - 3];
    q3 = ctrl_pts[N - 2];
    acc = (q1 + q3 - 2 * q2) / ts / ts;
    if (acc(0) < -ratio * limit_acc || acc(1) < -ratio * limit_acc ||
        acc(2) < -ratio * limit_acc) {
      q01 = q1 + (q1 - q0);
      q12 = 0.5 * (q1 + q2);
      q23 = q2 + (q2 - q3);
      qm = (1 / 6.0) * q01 + (2 / 3.0) * q12 + (1 / 6.0) * q23;
      ctrl_pts.insert(ctrl_pts.begin() + N - 3, qm);
      weight_vec.insert(weight_vec.begin() + N - 3, 0);
      double avg = (weight_vec[N - 4] + weight_vec[N - 2]) / 3.0;
      weight_vec[N - 4] = avg;
      weight_vec[N - 3] = avg;
      weight_vec[N - 2] = avg;
    }
  }

  /* convert vector to matrix*/
  control_pts.resize(ctrl_pts.size(), 3);
  for (int i = 0; i < ctrl_pts.size(); ++i) {
    control_pts.row(i) = ctrl_pts[i];
  }
  weight.resize(weight_vec.size());
  for (int i = 0; i < weight_vec.size(); ++i) {
    weight(i) = weight_vec[i];
  }

  cout << "dist cost weight: " << weight.transpose() << endl;
}

// input :
//      sample : 3 x (K+6) (for 5 order) for x, y, z sample
//      ts
// output:
//      control_pts (K+6)x3
void getControlPointEqu5(Eigen::MatrixXd samples, double ts,
                         Eigen::MatrixXd& control_pts) {
  int K = samples.cols() - 6;

  // write A
  Eigen::VectorXd prow(5), vrow(5), arow(5);
  prow << 1, 26, 66, 26, 1;
  vrow << -1, -10, 0, 10, 1;
  arow << 1, 2, -6, 2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 6, K + 6);

  for (int i = 0; i < K + 2; ++i) A.block(i, i, 1, 5) = prow.transpose();
  A.block(0, 0, K + 2, K + 6) = (1 / 120.0) * A.block(0, 0, K + 2, K + 6);

  A.block(K + 2, 0, 1, 5) = A.block(K + 3, K + 1, 1, 5) = vrow.transpose();
  A.block(K + 2, 0, 2, K + 6) = (1 / 24.0 / ts) * A.block(K + 2, 0, 2, K + 6);

  A.block(K + 4, 0, 1, 5) = A.block(K + 5, K + 1, 1, 5) = arow.transpose();
  A.block(K + 4, 0, 2, K + 6) =
      (1 / 6.0 / ts / ts) * A.block(K + 4, 0, 2, K + 6);

  // write b
  Eigen::VectorXd bx(K + 6), by(K + 6), bz(K + 6);
  for (int i = 0; i < K + 6; ++i) {
    bx(i) = samples(0, i);
    by(i) = samples(1, i);
    bz(i) = samples(2, i);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  control_pts.resize(K + 6, 3);
  control_pts.col(0) = px;
  control_pts.col(1) = py;
  control_pts.col(2) = pz;
}

// input: K: segment, N: sample num, ts: segment time, samples 3x(K+1)*(N+1)
// output: control_pts of b-spline: (K+6)x3
void getControlPointLeastSquare(int K, int N, double ts,
                                Eigen::MatrixXd samples,
                                Eigen::MatrixXd& control_pts) {
  cout << "K:" << K << endl;

  // write matrix block Ab of A
  Eigen::MatrixXd Ab(N + 1, 6);
  Ab << 0.00833333333333333, 0.216666666666667, 0.55, 0.216666666666667,
      0.00833333333333333, 0.0, 0.0019775390625, 0.124910481770833,
      0.519645182291667, 0.328076171875, 0.0253824869791667, 0.0,
      0.000260416666666667, 0.06171875, 0.438020833333333, 0.438020833333333,
      0.06171875, 0.000260416666666667, 0, 0.0253824869791667, 0.328076171875,
      0.519645182291667, 0.124910481770833, 0.0019775390625, 0.0,
      0.00833333333333331, 0.216666666666667, 0.55, 0.216666666666667,
      0.00833333333333333;
  // cout << "Ab:" << Ab << endl;

  // write A
  Eigen::MatrixXd A((N + 1) * (K + 1), K + 6);
  A.setZero();
  for (int i = 0; i <= K; ++i) {
    A.block((N + 1) * i, i, N + 1, 6) = Ab;
  }
  // cout << "A" << A << endl;

  // solve px, py, pz
  // Eigen::VectorXd px = A.bdcSvd(ComputeThinU | ComputeThinV).solve(bx);
  // Eigen::VectorXd py = A.bdcSvd(ComputeThinU | ComputeThinV).solve(by);
  // Eigen::VectorXd pz = A.bdcSvd(ComputeThinU | ComputeThinV).solve(bz);
  Eigen::VectorXd px =
      A.colPivHouseholderQr().solve(samples.row(0).transpose());
  Eigen::VectorXd py =
      A.colPivHouseholderQr().solve(samples.row(1).transpose());
  Eigen::VectorXd pz =
      A.colPivHouseholderQr().solve(samples.row(2).transpose());
  // cout << "px:" << px.transpose() << ", py:" << py.transpose() << ", pz:"
  // << pz.transpose() << endl;

  control_pts.resize(K + 6, 3);
  control_pts.col(0) = px;
  control_pts.col(1) = py;
  control_pts.col(2) = pz;
}

#endif