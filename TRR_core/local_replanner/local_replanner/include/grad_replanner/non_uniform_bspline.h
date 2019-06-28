#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <iostream>
#include "grad_replanner/utils/font_color.h"
using namespace std;

class NonUniformBspline {
 private:
  /* non-uniform bspline */
  int p, n, m;
  Eigen::MatrixXd control_points;
  Eigen::VectorXd u;  // knots vector
  double interval;    // init interval

  double limit_vel;
  double limit_acc;
  double limit_ratio;

  Eigen::Vector3d x0, v0, a0;

  Eigen::VectorXd getU(double u);
  Eigen::MatrixXd getPi(int idx);
  Eigen::MatrixXd getDerivativeControlPoints();

 public:
  NonUniformBspline() {}
  NonUniformBspline(Eigen::MatrixXd points, int order, double interval,
                    bool zero = true);
  ~NonUniformBspline();

  void setParameterAuto();
  void setDynamicsLimit(double vel, double acc);

  void setKnot(Eigen::VectorXd knot);
  Eigen::VectorXd getKnot();

  Eigen::MatrixXd getControlPoint() { return control_points; }

  void getRegion(double &um, double &um_p);

  Eigen::Vector3d evaluateDeBoor(double t);

  NonUniformBspline getDerivative();

  static void getControlPointEqu3(Eigen::MatrixXd samples, double ts,
                                  Eigen::MatrixXd &control_pts);
  static void BsplineParameterize(
      const double &ts, const vector<Eigen::Vector3d> &point_set,
      const vector<Eigen::Vector3d> &start_end_derivative,
      Eigen::MatrixXd &ctrl_pts);

  /* check feasibility, reallocate time and recompute first 3 ctrl pts */
  bool checkFeasibility(bool show = false);
  bool reallocateTime(bool show = false);
  void adjustTime();
  void adjustKnot(int num1, int num2, double ratio = 1.1);
  void recomputeInit();

  /* for evaluation */
  double getTimeSum();
  double getLength();
  double getJerk();

  void getMeanAndMaxVel(double &mean_v, double &max_v);
  void getMeanAndMaxAcc(double &mean_a, double &max_a);
};

#endif