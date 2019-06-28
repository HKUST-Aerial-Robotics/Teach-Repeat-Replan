#ifndef _TIME_DATA_TYPE_
#define _TIME_DATA_TYPE_

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct timeAllocator
{     
      double s_step; // step size of the parameter in the virtual domain
      //int K;         // discretion number of the grid in s from 0 ~ 1 
      int seg_num;   // segment number of all pieces of trajectory
      int K_max;
      double vel_m, acc_m, jer_m; // maximum limit of velocity, acceleration and jerk, in x,y,z axis 

      MatrixXd a, b, c, d; // result of the minimum time optimization program
      MatrixXd time;
      MatrixXd time_acc; // time divided by a in middle of each pair of b
      MatrixXd s;
      VectorXd K;

      timeAllocator( int _seg_num, double _s_step, int _K_max, double _vel_m, double _acc_m, double _jer_m)
      {     
            seg_num = _seg_num;
            s_step  = _s_step;
            K_max   = _K_max;

            vel_m = _vel_m;
            acc_m = _acc_m;
            jer_m = _jer_m;

            K = Eigen::VectorXd::Zero(seg_num);
            a = Eigen::MatrixXd::Zero(seg_num, K_max);
            b = Eigen::MatrixXd::Zero(seg_num, K_max + 1);
            c = Eigen::MatrixXd::Zero(seg_num, K_max + 1);
            d = Eigen::MatrixXd::Zero(seg_num, K_max);
            
            time = Eigen::MatrixXd::Zero(seg_num, K_max);
            time_acc = Eigen::MatrixXd::Zero(seg_num, K_max);
            s = Eigen::MatrixXd::Zero(seg_num, K_max + 1);
      }
      
      timeAllocator(){}
      ~timeAllocator(){}
};
#endif