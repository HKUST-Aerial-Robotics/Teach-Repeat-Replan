/*
This header file is used to provide some basic mathematic support for the Bernstein-basis trajectory generation optimization problem. Includes:
1-: Mapping matrix maps the coefficients of the Bernstein basis (ie. control points) to Monomial basis. The mapping matrix range from order 3 to order 10
2-: Modulus list of the Bernstein basis to a given order. That is, pre-compute the constant-modulus (the 'n choose k' combinatorial) of the basis std::vector. 
	To save computation cost of frequently call this value. 

The class should be initialized to a instance before the trajectory generator called. 
Several initializer are provided, and the instance is initialized according to the given order of the control points.
*/
#ifndef _BEZIER_BASE_H_
#define _BEZIER_BASE_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>

class Bernstein
{
	private:
		
		std::vector<Eigen::MatrixXd> MQMList_l, MQMList_u, MList, MQMList_arc, FMList;
		std::vector<Eigen::VectorXd> CList, CvList, CaList, CjList;

		Eigen::MatrixXd FM, MQM_l, MQM_u, M, MQM_arc;
		Eigen::VectorXd C, Cv, Ca, Cj;

		int order_min = 3, order_max = 12;  // The order of the polynomial in each segment, also the number of control points used in each segment
		int traj_order, ctrl_num1D;
		double min_order;                    // The order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap    

	public:
		Bernstein(){}; // Empty Constructor
		Bernstein(double min_order); // Constructor with fixed objective order
		~Bernstein(){};

		//void setOrder(double min_order);
		Eigen::MatrixXd CholeskyDecomp(Eigen::MatrixXd Q); // return square root F of Q; Q = F' * F

		std::vector<Eigen::MatrixXd> getMs(){ return MList; };
		std::vector<Eigen::MatrixXd> getMQM_ls(){ return MQMList_l; };
		std::vector<Eigen::MatrixXd> getMQM_us(){ return MQMList_u; };
		std::vector<Eigen::MatrixXd> getMQM_arcs(){ return MQMList_arc; };
		std::vector<Eigen::MatrixXd> getFMs(){ return FMList; };
		std::vector<Eigen::VectorXd> getCs(){ return CList; };
		std::vector<Eigen::VectorXd> getC_vs(){ return CvList; };
		std::vector<Eigen::VectorXd> getC_as(){ return CaList; };
		std::vector<Eigen::VectorXd> getC_js(){ return CjList; };

		Eigen::MatrixXd getM(){ return M; };
		Eigen::MatrixXd getMQM_l(){ return MQM_l; };
		Eigen::MatrixXd getMQM_u(){ return MQM_u; };
		Eigen::MatrixXd getMQM_arc(){ return MQM_arc; };
		Eigen::MatrixXd getFM(){ return FM; };
		Eigen::VectorXd getC(){ return C; };
		Eigen::VectorXd getC_v(){ return Cv; };
		Eigen::VectorXd getC_a(){ return Ca; };
		Eigen::VectorXd getC_j(){ return Cj; };

		void setFixedOrder(int traj_order_)
		{
			traj_order = traj_order_;
		    ctrl_num1D = traj_order + 1;

			MQM_l = getMQM_ls()[traj_order];
		    MQM_u = getMQM_us()[traj_order];
		    C     = getCs()  [traj_order];
		    Cv    = getC_vs()[traj_order];
		    Ca    = getC_as()[traj_order];
		    Cj    = getC_js()[traj_order];
		};

		inline Eigen::Vector3d getPos(const Eigen::MatrixXd & Coeff, const int & k, const double & s)
		{
		    Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);

		    for(int i = 0; i < 3; i++)
		        for(int j = 0; j < ctrl_num1D; j++)
		            ret(i) += C(j) * Coeff(k, i * ctrl_num1D + j) * pow(s, j) * pow((1 - s), (traj_order - j) ); 

		    return ret;
		};


		inline Eigen::Vector3d getVel(const Eigen::MatrixXd & Coeff, const int & k, const double & s)
		{
		    Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);

		    for(int i = 0; i < 3; i++)
		        for(int j = 0; j < ctrl_num1D; j++)
		            if(j < ctrl_num1D - 1 )
		                ret(i) += Cv(j) * traj_order 
		                        * ( Coeff(k, i * ctrl_num1D + j + 1) - Coeff(k, i * ctrl_num1D + j))
		                          * pow(s, j) * pow((1 - s), (traj_order - j - 1) ); 

		    return ret;
		};

		inline Eigen::Vector3d getAcc(const Eigen::MatrixXd & Coeff, const int & k, const double & s)
		{
		    Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);

		    for(int i = 0; i < 3; i++)
		        for(int j = 0; j < ctrl_num1D; j++)
		            if(j < ctrl_num1D - 2 )
		              ret(i) += Ca(j) * traj_order * (traj_order - 1) 
		                        * ( Coeff(k, i * ctrl_num1D + j + 2) - 2 * Coeff(k, i * ctrl_num1D + j + 1) + Coeff(k, i * ctrl_num1D + j))
		                          * pow(s, j) * pow((1 - s), (traj_order - j - 2) );                         

		    return ret;
		};

		inline Eigen::Vector3d getPosFromBezier(const Eigen::MatrixXd & polyCoeff, const double & t_now, const int & seg_now )
		{
		    Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);

		    for(int i = 0; i < 3; i++)
		        for(int j = 0; j < ctrl_num1D; j++)
		            ret(i) += C(j) * polyCoeff(seg_now, i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (traj_order - j) ); 

		    return ret;  
		};
};

#endif