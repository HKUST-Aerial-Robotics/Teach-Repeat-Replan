#ifndef _TIME_OPTIMIZER_H_
#define _TIME_OPTIMIZER_H_

#include <Eigen/Dense>
#include <mosek.h>
#include <global_planner/utils/time_allocator.h>
#include <global_planner/utils/bezier_base.h>

class temporalTrajOptimizer 
{
    private:
        Eigen::MatrixXd _P; // recording the polynomial's coefficients for further evaluation
        Eigen::VectorXd _T; // recording the time allocation for further evaluation

        int _seg_num, _poly_num1D;

        int _poly_order, _ctrl_num1D;
        VectorXd _C, _Cv, _Ca;
        int _type = 0; // 0 for poly traj; 1 for bezier traj

        double _objective;
        timeAllocator * time_allocator; // for return the final result to high-level planer

        Eigen::Vector3d getVel(int k, double s);
        Eigen::Vector3d getAcc(int k, double s);

        Eigen::Vector3d getVelPoly(int k, double s);
        Eigen::Vector3d getAccPoly(int k, double s);

        Eigen::Vector3d getVelBezier(int k, double s);
        Eigen::Vector3d getAccBezier(int k, double s);

    public:
        temporalTrajOptimizer(){};
        ~temporalTrajOptimizer(){};

        void timeProfileGeneration( const Eigen::MatrixXd & polyCoeff, // the polynomial coefficients in virtual domain form 0.0 to 1.0
                                          Eigen::VectorXd & time,
                                    const double & maxVel,      // maximum phisical limitation of v, a, and j
                                    const double & maxAcc,
                                    const double & maxdAcc,
                                    const double & d_s, 
                                    const double & w_a );   // discretize size of a grid in s from 0.0 to 1.0

        timeAllocator * getTimeProfile() {return time_allocator;}
        
        void setParam( int poly_order, Eigen::VectorXd C, Eigen::VectorXd Cv, Eigen::VectorXd Ca)
        { 
            _poly_order = poly_order; 
            _ctrl_num1D = poly_order + 1;
            _C  = C;
            _Cv = Cv;
            _Ca = Ca;
        }

        void setType(int type)
        {
            _type = type;
        }
};

#endif
