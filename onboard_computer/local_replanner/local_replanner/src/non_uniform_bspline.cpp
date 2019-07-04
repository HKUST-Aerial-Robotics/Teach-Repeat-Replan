#include "grad_replanner/non_uniform_bspline.h"
#include <ros/ros.h>

// control points is a (n+1)x3 matrix
NonUniformBspline::NonUniformBspline(Eigen::MatrixXd points, int order,
                                     double interval, bool zero) {
  this->p = order;

  control_points = points;
  this->n = points.rows() - 1;

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

  // show the result
  // cout << "p: " << p << "  n: " << n << "  m: " << m << endl;
  // cout << "control pts:\n" << control_points << "\nknots:\n" <<
  // this->u.transpose() << endl; cout << "M3:\n" << M[0] << "\nM4:\n" << M[1]
  // << "\nM5:\n" << M[2] << endl;
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setParameterAuto() {
  ros::param::get("/local_replanner/limit_ratio", limit_ratio);
  ros::param::get("/local_replanner/max_vel", limit_vel);
  ros::param::get("/local_replanner/max_acc", limit_acc);
}

void NonUniformBspline::setDynamicsLimit(double vel, double acc) {
  limit_vel = vel;
  limit_acc = acc;
}

void NonUniformBspline::setKnot(Eigen::VectorXd knot) { this->u = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u; }

void NonUniformBspline::getRegion(double &um, double &um_p) {
  um = this->u(this->p);
  um_p = this->u(this->m - this->p);
}

Eigen::VectorXd NonUniformBspline::getU(double u) {
  Eigen::VectorXd uv = Eigen::VectorXd::Zero(this->p + 1);
  uv(0) = 1.0;
  for (int i = 1; i <= this->p; ++i) {
    uv(i) = uv(i - 1) * u;
  }
  return uv;
}

Eigen::MatrixXd NonUniformBspline::getPi(int idx) {
  Eigen::MatrixXd pi = control_points.block(idx - p, 0, p + 1, 3);
  return pi;
}

Eigen::Vector3d NonUniformBspline::evaluateDeBoor(double t) {
  if (t < this->u(this->p) || t > this->u(this->m - this->p)) {
    //cout << "Out of trajectory range." << endl;
    // return Eigen::Vector3d::Zero(3);
    
    if (t < this->u(this->p)) t = this->u(this->p);
    if (t > this->u(this->m - this->p)) t = this->u(this->m - this->p);
  }

  // determine which [ui,ui+1] lay in
  int k = this->p;
  while (true) {
    if (this->u(k + 1) >= t) break;
    ++k;
  }

  /* deBoor's alg */
  vector<Eigen::Vector3d> d;
  for (int i = 0; i <= p; ++i) {
    d.push_back(control_points.row(k - p + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p; ++r) {
    for (int i = p; i >= r; --i) {
      double alpha = (t - u[i + k - p]) / (u[i + 1 + k - r] - u[i + k - p]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p];
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p-1
  // control point Qi = p*(Pi+1-Pi)/(ui+p+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points.rows() - 1, 3);
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) = p * (control_points.row(i + 1) - control_points.row(i)) /
                 (u(i + p + 1) - u(i + 1));
  }
  return ctp;
}

NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd ctp = this->getDerivativeControlPoints();

  NonUniformBspline derivative =
      NonUniformBspline(ctp, p - 1, this->interval, false);

  /* cut the first and last knot */
  Eigen::VectorXd knot(this->u.rows() - 2);
  knot = this->u.segment(1, this->u.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points.rows() << REC;

  Eigen::MatrixXd P = control_points;

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::Vector3d vel =
        p * (P.row(i + 1) - P.row(i)) / (u(i + p + 1) - u(i + 1));
    if (fabs(vel(0)) > limit_vel + 0.1 || fabs(vel(1)) > limit_vel + 0.1 ||
        fabs(vel(2)) > limit_vel + 0.1) {
      /* insert mid point */
      if (show)
        SETR << "[Check]: Infeasible vel " << i << " :" << vel.transpose()
             << REC;
      fea = false;
      max_vel = max(max_vel, fabs(vel(0)));
      max_vel = max(max_vel, fabs(vel(1)));
      max_vel = max(max_vel, fabs(vel(2)));
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::Vector3d acc =
        p * (p - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u(i + p + 2) - u(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u(i + p + 1) - u(i + 1))) /
        (u(i + p + 1) - u(i + 2));

    if (fabs(acc(0)) > limit_acc + 0.1 || fabs(acc(1)) > limit_acc + 0.1 ||
        fabs(acc(2)) > limit_acc + 0.1) {
      /* insert mid point */
      if (show)
        SETR << "[Check]: Infeasible acc " << i << " :" << acc.transpose()
             << REC;
      fea = false;
      max_acc = max(max_acc, fabs(acc(0)));
      max_acc = max(max_acc, fabs(acc(1)));
      max_acc = max(max_acc, fabs(acc(2)));

      double ma = max(fabs(acc(0)), fabs(acc(1)));
      ma = max(ma, fabs(acc(2)));

      double ratio = sqrt(ma / limit_acc) + 1e-4;
      // cout << "ratio: " << ratio << endl;
    }
  }

  // cout << "max vel:" << max_vel << ", max acc:" << max_acc << endl;
  double ratio = max(max_vel / limit_vel, sqrt(fabs(max_acc) / limit_acc));
  //SETY << "check ratio:" << ratio << REC;

  return fea;
}

bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points.rows() << REC;
  // cout << "origin knots:\n" << u.transpose() << endl;
  bool fea = true;

  // double tm, tmp;
  // getRegion(tm, tmp);
  // double to = tmp - tm;
  // cout << "origin duration: " << to << endl;

  Eigen::MatrixXd P = control_points;

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  const int der_num_v = 4;
  const int der_num_a = 3;

  for (int i = der_num_v; i < P.rows() - 1 - der_num_v; ++i) {
    Eigen::Vector3d vel =
        p * (P.row(i + 1) - P.row(i)) / (u(i + p + 1) - u(i + 1));
    if (fabs(vel(0)) > limit_vel + 1e-4 || fabs(vel(1)) > limit_vel + 1e-4 ||
        fabs(vel(2)) > limit_vel + 1e-4) {
      fea = false;
      max_vel = -1.0;
      max_vel = max(max_vel, fabs(vel(0)));
      max_vel = max(max_vel, fabs(vel(1)));
      max_vel = max(max_vel, fabs(vel(2)));
      if (show)
        SETR << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose()
             << REC;

      double ratio = max_vel / limit_vel + 1e-4;
      if (ratio > limit_ratio) ratio = limit_ratio;

      double time_ori = u(i + p + 1) - u(i + 1);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p);

      for (int j = i + 2; j <= i + p + 1; ++j) {
        u(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      for (int j = i + p + 2; j < u.rows(); ++j) {
        u(j) += delta_t;
      }
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = der_num_a; i < P.rows() - 2 - der_num_a; ++i) {
    Eigen::Vector3d acc =
        p * (p - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u(i + p + 2) - u(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u(i + p + 1) - u(i + 1))) /
        (u(i + p + 1) - u(i + 2));

    if (fabs(acc(0)) > limit_acc + 1e-4 || fabs(acc(1)) > limit_acc + 1e-4 ||
        fabs(acc(2)) > limit_acc + 1e-4) {
      fea = false;
      /* insert mid point */
      max_acc = -1.0;
      max_acc = max(max_acc, fabs(acc(0)));
      max_acc = max(max_acc, fabs(acc(1)));
      max_acc = max(max_acc, fabs(acc(2)));
      if (show)
        SETR << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose()
             << REC;

      double ratio = sqrt(max_acc / limit_acc) + 1e-4;
      if (ratio > limit_ratio) ratio = limit_ratio;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u(i + p + 1) - u(i + 2);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p - 1);

      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u.rows(); ++j) {
          u(j) += 4.0 * t_inc;
        }
      } else {
        for (int j = i + 3; j <= i + p + 1; ++j) {
          u(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p + 2; j < u.rows(); ++j) {
          u(j) += delta_t;
        }
      }
    }
  }

  // cout << "new knots:\n" << u.transpose() << endl;
  // getRegion(tm, tmp);
  // double tn = tmp - tm;
  // cout << "new duration: " << tn << endl;
  // SETY << "realloc ratio: " << tn / to << REC;

  return fea;
}

void NonUniformBspline::adjustTime() {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;
  adjustKnot(num1, num2, 1.05);
}

void NonUniformBspline::adjustKnot(int num1, int num2, double ratio) {
  double delta_t = (ratio - 1.0) * (u(num2) - u(num1));
  double t_inc = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u.rows(); ++i) u(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {
  double t1 = u(1), t2 = u(2), t3 = u(3), t4 = u(4), t5 = u(5);

  /* write the A matrix */
  Eigen::Matrix3d A;

  /* position */
  A(0, 0) =
      ((t2 - t5) * (t3 - t4) * (t3 - t4)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(0, 1) =
      ((t2 - t5) * (t3 - t4) * (t1 - t3)) /
          ((t1 - t4) * (t2 - t4) * (t2 - t5)) +
      ((t1 - t4) * (t2 - t3) * (t3 - t5)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(0, 2) =
      ((t1 - t4) * (t2 - t3) * (t2 - t3)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));

  /* velocity */
  A(1, 0) = 3.0 * ((t2 - t5) * (t3 - t4)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(1, 1) = 3.0 * ((t1 - t4) * (t2 - t3) - (t2 - t5) * (t3 - t4)) /
            ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(1, 2) =
      -3.0 * ((t1 - t4) * (t2 - t3)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));

  /* acceleration */
  A(2, 0) = 6.0 * (t2 - t5) / ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(2, 1) =
      -6.0 * ((t1 - t4) + (t2 - t5)) / ((t1 - t4) * (t2 - t4) * (t2 - t5));
  A(2, 2) = 6.0 * (t1 - t4) / ((t1 - t4) * (t2 - t4) * (t2 - t5));

  /* write B = (bx, by, bz) */
  Eigen::Matrix3d B;
  Eigen::Vector3d bx, by, bz;
  B.row(0) = x0;
  B.row(1) = v0;
  B.row(2) = a0;
  // cout << "B:\n" << B << endl;

  bx = B.col(0);
  by = B.col(1);
  bz = B.col(2);

  /* solve */
  Eigen::Vector3d px = A.colPivHouseholderQr().solve(bx);
  Eigen::Vector3d py = A.colPivHouseholderQr().solve(by);
  Eigen::Vector3d pz = A.colPivHouseholderQr().solve(bz);

  Eigen::Matrix3d P;
  P.col(0) = px;
  P.col(1) = py;
  P.col(2) = pz;

  control_points.row(0) = P.row(0);
  control_points.row(1) = P.row(1);
  control_points.row(2) = P.row(2);

  B = A * P;
  // cout << "B:\n" << B << endl;
}

// input :
//      sample : 3 x (K+2) (for 3 order) for x, y, z sample
//      ts
// output:
//      control_pts (K+6)x3
void NonUniformBspline::getControlPointEqu3(Eigen::MatrixXd samples, double ts,
                                            Eigen::MatrixXd &control_pts) {
  int K = samples.cols() - 4 - 1;

  // write A
  Eigen::VectorXd prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 5, K + 4);

  for (int i = 0; i < K + 2; ++i) A.block(i, i, 1, 3) = prow.transpose();

  A.block(K + 2, 0, 1, 3) = A.block(K + 3, K + 1, 1, 3) = vrow.transpose();
  A.block(K + 4, 0, 1, 3) = arow.transpose();

  // cout << "A:\n" << A << endl;
  A.block(0, 0, K + 2, K + 4) = (1 / 6.0) * A.block(0, 0, K + 2, K + 4);
  A.block(K + 2, 0, 2, K + 4) = (1 / 2.0 / ts) * A.block(K + 2, 0, 2, K + 4);
  A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);

  // write b
  Eigen::VectorXd bx(K + 5), by(K + 5), bz(K + 5);
  for (int i = 0; i < K + 5; ++i) {
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

void NonUniformBspline::BsplineParameterize(
    const double &ts, const vector<Eigen::Vector3d> &point_set,
    const vector<Eigen::Vector3d> &start_end_derivative,
    Eigen::MatrixXd &ctrl_pts) {
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() <= 3) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points."
         << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size() - 2;

  // write A
  Eigen::VectorXd prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 6, K + 4);

  for (int i = 0; i < K + 2; ++i) A.block(i, i, 1, 3) = prow.transpose();
  A.block(K + 2, 0, 1, 3) = A.block(K + 3, K + 1, 1, 3) = vrow.transpose();
  A.block(K + 4, 0, 1, 3) = A.block(K + 5, K + 1, 1, 3) = arow.transpose();

  // cout << "A:\n" << A << endl;
  A.block(0, 0, K + 2, K + 4) = (1 / 6.0) * A.block(0, 0, K + 2, K + 4);
  A.block(K + 2, 0, 2, K + 4) = (1 / 2.0 / ts) * A.block(K + 2, 0, 2, K + 4);
  A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  Eigen::VectorXd bx(K + 6), by(K + 6), bz(K + 6);
  for (int i = 0; i < K + 2; ++i) {
    bx(i) = point_set[i](0), by(i) = point_set[i](1), bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + 2 + i) = start_end_derivative[i](0);
    by(K + 2 + i) = start_end_derivative[i](1);
    bz(K + 2 + i) = start_end_derivative[i](2);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 4, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  //cout << "[B-spline]: parameterization ok." << endl;
}

double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getRegion(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength() {
  double length = 0.0;

  double tm, tmp;
  getRegion(tm, tmp);
  Eigen::Vector3d p_l = evaluateDeBoor(tm), p_n;
  for (double t = tm + 0.01; t <= tmp; t += 0.01) {
    p_n = evaluateDeBoor(t);
    length += (p_n - p_l).norm();
    p_n = p_l;
  }

  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();
  Eigen::VectorXd times = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();

  cout << "num knot:" << times.rows() << endl;
  cout << "num ctrl pts:" << ctrl_pts.rows() << endl;

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    jerk += (times(i + 1) - times(i)) * ctrl_pts(i, 0) * ctrl_pts(i, 0);
    jerk += (times(i + 1) - times(i)) * ctrl_pts(i, 1) * ctrl_pts(i, 1);
    jerk += (times(i + 1) - times(i)) * ctrl_pts(i, 2) * ctrl_pts(i, 2);
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v) {
  NonUniformBspline vel = getDerivative();
  double tm, tmp;
  vel.getRegion(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d v3d = vel.evaluateDeBoor(t);
    double vn = v3d.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v = mean_vel;
  max_v = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double tm, tmp;
  acc.getRegion(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d a3d = acc.evaluateDeBoor(t);
    double an = a3d.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a = mean_acc;
  max_a = max_acc;
}