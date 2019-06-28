#ifndef _BSPLINE_REPLANNER_H_
#define _BSPLINE_REPLANNER_H_

#include "grad_replanner/grad_band_optimizer.h"
#include "grad_replanner/non_uniform_bspline.h"
#include "local_perception/sdf_map.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>

class BsplineReplanner {
 public:
  BsplineReplanner(/* args */);
  ~BsplineReplanner();

  void initialize(double max_v, double max_a);

  void setInput(const shared_ptr<SDFMap>& sdf_map, const double dt,
                const vector<Eigen::Vector3d>& pt_set,
                const vector<Eigen::Vector3d>& start_end_derivative);

  void renewLambda1(double lamda1_);
  void renewLambda2(double lamda2_);

  void resetLambda2();

  bool optimize(bool adjust_time);

  NonUniformBspline getTraj();

  vector<Eigen::Vector3d> getAccControlPts();
  vector<Eigen::Vector3d> getJerkControlPts();

 private:
  vector<Eigen::Vector3d> point_set, start_end_derivative;
  double delta_t, length_ori, vel_limit, acc_limit;
  int num_seg_ori, num_seg_opt;

  unique_ptr<GradBandOptimizer> traj_optimizer;
  NonUniformBspline traj;

  double getTrajLength(const vector<Eigen::Vector3d>& pt_set);
  double getTrajLength(const Eigen::MatrixXd& pt_set);
};

#endif