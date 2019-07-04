#ifndef _TRAJECTORY_GENERATOR_OOQP_H_
#define _TRAJECTORY_GENERATOR_OOQP_H_

#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>

#include <global_planner/utils/data_type.h>
#include <global_planner/utils/bezier_base.h>

class spatialTrajOptimizer 
{
    private:
        double obj;
        Eigen::MatrixXd PolyCoeff;
        Eigen::VectorXd PolyTime;

    public:
        spatialTrajOptimizer(){}
        ~spatialTrajOptimizer(){}

        int bezierCurveGeneration( 
        const decomp_cvx_space::FlightCorridor &corridor,
        const Eigen::MatrixXd &MQM_u,
        const Eigen::MatrixXd &MQM_l,
        const Eigen::MatrixXd &pos,
        const Eigen::MatrixXd &vel,
        const Eigen::MatrixXd &acc,
        const int traj_order,
        const double minimize_order,
        const double max_vel, 
        const double max_acc);

        Eigen::MatrixXd getPolyCoeff()
        {
            return PolyCoeff;
        };

        Eigen::VectorXd getPolyTime()
        {
            return PolyTime;
        };

        double getObjective()
        {
            return obj;
        };
};

#endif