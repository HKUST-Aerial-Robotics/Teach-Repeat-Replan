#include <global_planner/spatial_optimizer.h>
using namespace std;    
using namespace Eigen;

int spatialTrajOptimizer::bezierCurveGeneration( 
        const decomp_cvx_space::FlightCorridor &corridor,
        const MatrixXd &MQM_u,
        const MatrixXd &MQM_l,
        const MatrixXd &pos,
        const MatrixXd &vel,
        const MatrixXd &acc,
        const int traj_order,
        const double minimize_order,
        const double maxVel, 
        const double maxAcc) 
{   
#define ENFORCE_VEL 0
#define ENFORCE_ACC 0 

    vector<decomp_cvx_space::Polytope> polyhedrons  = corridor.polyhedrons;
    vector<double>   time = corridor.durations;

    int segment_num  = polyhedrons.size();
    double initScale = time[0];
    double lstScale  = time[segment_num- 1];

    int s1d1CtrlP_num = traj_order + 1;
    int s1CtrlP_num   = 3 * s1d1CtrlP_num;

    int equ_con_s_num = 3 * 3; // start state
    int equ_con_e_num = 3 * 3; // end state
    int equ_con_continuity_num = 3 * 3 * (segment_num - 1);
    int equ_con_num = equ_con_s_num + equ_con_e_num + equ_con_continuity_num;  // p, v, a in x, y, z axis in each segment's joint position

    int ieq_con_pos_num = 0; // all control points within the polytopes, each face assigned a linear constraint
    for(int i = 0; i < segment_num; i++)
    {
        decomp_cvx_space::Polytope pltp = corridor.polyhedrons[i];
        for(int j = 0; j < (int)pltp.planes.size(); j++ )
        {
            ieq_con_pos_num += s1d1CtrlP_num;
        }
    }

    int vel_con_num = 3 *  traj_order * segment_num;
    int acc_con_num = 3 * (traj_order - 1) * segment_num;

    if( !ENFORCE_VEL )
        vel_con_num = 0;

    if( !ENFORCE_ACC )
        acc_con_num = 0;

    int high_order_con_num = vel_con_num + acc_con_num;     

    int ctrlP_num = segment_num * s1CtrlP_num;
    vector< double > con_eq_bd; 
    vector< pair<double, double> > con_ie_bd; 

    for(int i = 0; i < equ_con_num; i ++ )
    { 
        double beq_i;
        if(i < 3)                    beq_i = pos(0, i); 
        else if (i >= 3  && i < 6  ) beq_i = vel(0, i - 3 ); 
        else if (i >= 6  && i < 9  ) beq_i = acc(0, i - 6 );
        else if (i >= 9  && i < 12 ) beq_i = pos(1, i - 9 );
        else if (i >= 12 && i < 15 ) beq_i = vel(1, i - 12);
        else if (i >= 15 && i < 18 ) beq_i = acc(1, i - 15);
        else beq_i = 0.0;

        con_eq_bd.push_back(beq_i);
    }
    
    if(ENFORCE_VEL)
    {
        for(int i = 0; i < vel_con_num; i++)
        {
            pair<double, double> cb_ie = make_pair( - maxVel,  + maxVel);
            con_ie_bd.push_back(cb_ie);   
        }
    }

    if(ENFORCE_ACC)
    {
        for(int i = 0; i < acc_con_num; i++)
        {
            pair<double, double> cb_ie = make_pair( - maxAcc, + maxAcc); 
            con_ie_bd.push_back(cb_ie);   
        }
    }

//  ### setting bounds on optimized variables
    const int nx = ctrlP_num;
    // upper and lower bounds for all unknowns x
    double  xupp[nx];    
    char   ixupp[nx];
    double  xlow[nx];
    char   ixlow[nx];

    // stacking all bnounds
    int n_idx = 0;
    for(int k = 0; k < segment_num; k++)
    {   
        for(int dim = 0; dim < 3; dim ++)
        {   
            for(int i = 0; i < s1d1CtrlP_num; i++)
            {   
                ixlow[n_idx] = 0;
                ixupp[n_idx] = 0;
                xlow[n_idx]  = 0;
                xupp[n_idx]  = 0;
                n_idx ++;
            }
        }
    }

    int my = equ_con_num; // equality constarins

    double b[my];
    for(int i = 0; i < my; i++)
        b[i] = con_eq_bd[i];
   
    int nn_idx  = 0;
    int row_idx = 0;
   
    int nnzA  = (1 * 3 + 2 * 3 + 3 * 3) * 2 + (segment_num - 1) * (2 + 4 + 6) * 3;

    double dA[nnzA];
    int irowA[nnzA];
    int jcolA[nnzA];

    // stacking all equality constraints
    //   Start position 
    {
        // position :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z       
            dA[nn_idx] = 1.0 * initScale;
            irowA[nn_idx] = row_idx;
            jcolA[nn_idx] = i * s1d1CtrlP_num;
            row_idx ++;
            nn_idx  ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   = - 1.0 * traj_order;
            dA[nn_idx+1] =   1.0 * traj_order;
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;

            jcolA[nn_idx]   = i * s1d1CtrlP_num;
            jcolA[nn_idx+1] = i * s1d1CtrlP_num + 1;

            row_idx ++;
            nn_idx += 2;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   =   1.0 * traj_order * (traj_order - 1) / initScale;
            dA[nn_idx+1] = - 2.0 * traj_order * (traj_order - 1) / initScale;
            dA[nn_idx+2] =   1.0 * traj_order * (traj_order - 1) / initScale;
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;
            irowA[nn_idx+2] = row_idx;

            jcolA[nn_idx]   = i * s1d1CtrlP_num;
            jcolA[nn_idx+1] = i * s1d1CtrlP_num + 1;
            jcolA[nn_idx+2] = i * s1d1CtrlP_num + 2;

            row_idx ++;
            nn_idx += 3;
        }
    }      
    //   End position 
    {   
        // position :
        for(int i = 0; i < 3; i++)
        {   
            dA[nn_idx]  = 1.0 * lstScale;
            irowA[nn_idx] = row_idx;
            jcolA[nn_idx] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;

            row_idx ++;
            nn_idx  ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   = - 1.0 * traj_order;
            dA[nn_idx+1] =   1.0 * traj_order;
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;

            jcolA[nn_idx]   = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
            jcolA[nn_idx+1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;

            row_idx ++;
            nn_idx += 2;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   =   1.0 * traj_order * (traj_order - 1) / lstScale;
            dA[nn_idx+1] = - 2.0 * traj_order * (traj_order - 1) / lstScale;
            dA[nn_idx+2] =   1.0 * traj_order * (traj_order - 1) / lstScale;
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;
            irowA[nn_idx+2] = row_idx;

            jcolA[nn_idx]   = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 2;
            jcolA[nn_idx+1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
            jcolA[nn_idx+2] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num ;

            row_idx ++;
            nn_idx += 3;
        }
    }

    // joint points
    {
        int sub_shift = 0;
        double val0, val1;
        for(int k = 0; k < (segment_num - 1); k ++ )
        {   
            double scale_k = time[k];
            double scale_n = time[k+1];
            // position :
            val0 = scale_k;
            val1 = scale_n;
            for(int i = 0; i < 3; i++)
            {
                dA[nn_idx]   =  1.0 * val0;
                dA[nn_idx+1] = -1.0 * val1;
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;

                jcolA[nn_idx]   = sub_shift + (i+1) * s1d1CtrlP_num - 1;
                jcolA[nn_idx+1] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;

                row_idx ++;
                nn_idx += 2;
            }
            
            for(int i = 0; i < 3; i++)
            {  
                dA[nn_idx]   = -1.0;
                dA[nn_idx+1] =  1.0;
                dA[nn_idx+2] =  1.0;
                dA[nn_idx+3] = -1.0;
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                irowA[nn_idx+2] = row_idx;
                irowA[nn_idx+3] = row_idx;

                jcolA[nn_idx]   = sub_shift + (i+1) * s1d1CtrlP_num - 2;    
                jcolA[nn_idx+1] = sub_shift + (i+1) * s1d1CtrlP_num - 1;    
                jcolA[nn_idx+2] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
                jcolA[nn_idx+3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
                
                row_idx ++;
                nn_idx += 4;
            }
            // acceleration :
            val0 = 1.0 / scale_k;
            val1 = 1.0 / scale_n;
            for(int i = 0; i < 3; i++)
            {  
                dA[nn_idx]   =  1.0  * val0;
                dA[nn_idx+1] = -2.0  * val0;
                dA[nn_idx+2] =  1.0  * val0;
                dA[nn_idx+3] = -1.0  * val1;
                dA[nn_idx+4] =  2.0  * val1;
                dA[nn_idx+5] = -1.0  * val1;
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                irowA[nn_idx+2] = row_idx;
                irowA[nn_idx+3] = row_idx;
                irowA[nn_idx+4] = row_idx;
                irowA[nn_idx+5] = row_idx;

                jcolA[nn_idx]   = sub_shift + (i+1) * s1d1CtrlP_num - 3;    
                jcolA[nn_idx+1] = sub_shift + (i+1) * s1d1CtrlP_num - 2;    
                jcolA[nn_idx+2] = sub_shift + (i+1) * s1d1CtrlP_num - 1;    
                jcolA[nn_idx+3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;    
                jcolA[nn_idx+4] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
                jcolA[nn_idx+5] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 2;

                row_idx ++;
                nn_idx += 6;
            }

            sub_shift += s1CtrlP_num;
        }
    }

    // linear constraints: hyperplanes; boundary of acceleration and velocity
    const int mz  = ieq_con_pos_num + high_order_con_num;// + 3 * segment_num * (s1d1CtrlP_num - 2) + 3 * segment_num * (s1d1CtrlP_num - 1);

    char iclow[mz];
    char icupp[mz];
    double clow[mz];
    double cupp[mz];

    int m_idx = 0;
    // stacking all bnounds
    for(int k = 0; k < segment_num; k++) 
    {   
        decomp_cvx_space::Polytope pltp = polyhedrons[k];
        int p_k = pltp.planes.size(); // hyperplane num of this polyhedra

        for(int i = 0; i < s1d1CtrlP_num; i++)
        {   
            for(int j = 0; j < p_k; j++)
            {   
                iclow[m_idx] = 0; 
                icupp[m_idx] = 1;

                clow[m_idx]  = 0;
                cupp[m_idx]  = - pltp.planes[j](3); // plane equation: ax + by + cz + K < 0 ==> ax + by + cz < -K

                m_idx ++;
            }
        }
    }

    // stack inequality constraints for velocity and acceleraion
    for(int i = 0; i < high_order_con_num; i++)
    {
        iclow[m_idx + i] = 1;
        icupp[m_idx + i] = 1;
        clow[m_idx + i]  = con_ie_bd[i].first;
        cupp[m_idx + i]  = con_ie_bd[i].second;
    }

    // add velocity     constraints on all segments of the trajectory
    // add acceleration constraints on all segments of the trajectory
    // linear constraints, equations
    int nnzC = 3 * ieq_con_pos_num; 
    if(ENFORCE_VEL) nnzC += 2 * traj_order * 3 * segment_num;
    if(ENFORCE_ACC) nnzC += 3 * (traj_order - 1) * 3 * segment_num;

    int irowC[nnzC];
    int jcolC[nnzC];
    double dC[nnzC];

    nn_idx  = 0;
    row_idx = 0;
    // gap constrains on position: in-equality constraints on the position
    for(int k = 0; k < segment_num; k++)
    {   
        decomp_cvx_space::Polytope pltp = polyhedrons[k];
        int p_k = pltp.planes.size(); // hyperplane num of this polyhedra

        double scale_k = time[k];
        double val0 = scale_k;

        for(int i = 0; i < s1d1CtrlP_num; i++)
        {  
            for(int j = 0; j < p_k; j++)
            {   
                dC[nn_idx]   = pltp.planes[j](0) * val0;
                dC[nn_idx+1] = pltp.planes[j](1) * val0;
                dC[nn_idx+2] = pltp.planes[j](2) * val0;
                
                irowC[nn_idx]   = row_idx;
                irowC[nn_idx+1] = row_idx;
                irowC[nn_idx+2] = row_idx;

                jcolC[nn_idx]   = k * s1CtrlP_num + i + 0 * s1d1CtrlP_num; // control point on x axis
                jcolC[nn_idx+1] = k * s1CtrlP_num + i + 1 * s1d1CtrlP_num; // control point on y axis
                jcolC[nn_idx+2] = k * s1CtrlP_num + i + 2 * s1d1CtrlP_num; // control point on z axis

                row_idx ++;
                nn_idx += 3;
            }
        }
    }

//  ######################################################
    // The velocity constraints
    if(ENFORCE_VEL)
    {   
        for(int k = 0; k < segment_num ; k ++ )
        {   
            for(int i = 0; i < 3; i++)
            {  // for x, y, z loop
                for(int p = 0; p < traj_order; p++)
                {
                    dC[nn_idx]     = -1.0 * traj_order;
                    dC[nn_idx + 1] =  1.0 * traj_order;

                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;

                    jcolC[nn_idx]     = k * s1CtrlP_num + i * s1d1CtrlP_num + p;    
                    jcolC[nn_idx + 1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;    

                    row_idx ++;
                    nn_idx += 2;
                }
            }
        }
    }

    // The acceleration constraints
    if(ENFORCE_ACC)
    {
        for(int k = 0; k < segment_num ; k ++ )
        {   
            double scale_k = time[k];
            for(int i = 0; i < 3; i++)
            { 
                for(int p = 0; p < traj_order - 1; p++)
                {    
                    dC[nn_idx]     =  1.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 1] = -2.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 2] =  1.0 * traj_order * (traj_order - 1) / scale_k;

                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;
                    irowC[nn_idx + 2] = row_idx;

                    jcolC[nn_idx]     = k * s1CtrlP_num + i * s1d1CtrlP_num + p;    
                    jcolC[nn_idx + 1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;    
                    jcolC[nn_idx + 2] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 2;    
                    
                    row_idx ++;
                    nn_idx += 3;
                }
            }
        }
    }

    // stack the objective function
    const int nnzQ = 3 * segment_num * (traj_order + 1) * (traj_order + 2) / 2;
    int    irowQ[nnzQ]; 
    int    jcolQ[nnzQ];
    double    dQ[nnzQ];

    double  c[nx];
    for(int i = 0; i < nx; i++)
        c[i] = 0.0;

    {   
        int min_order_l = floor(minimize_order);
        int min_order_u = ceil (minimize_order);

        int sub_shift = 0;
        int idx = 0;
        for(int k = 0; k < segment_num; k ++)
        {
            double scale_k = time[k];
            for(int p = 0; p < 3; p ++ )
                for( int i = 0; i < s1d1CtrlP_num; i ++ )
                    for( int j = 0; j < s1d1CtrlP_num; j ++ )
                        if( i >= j )
                        {
                            irowQ[idx] = sub_shift + p * s1d1CtrlP_num + i;   
                            jcolQ[idx] = sub_shift + p * s1d1CtrlP_num + j;  
            
                            if(min_order_l == min_order_u)
                                dQ[idx] = MQM_u(i, j) / (double)pow(scale_k, 2 * min_order_u - 3) * pow(corridor.scale_factor, 2 * min_order_u - 1);
                            else
                                dQ[idx] = MQM_l(i, j) / (double)pow(scale_k, 2 * min_order_l - 3) * pow(corridor.scale_factor, 2 * min_order_l - 1)
                                        + MQM_u(i, j) / (double)pow(scale_k, 2 * min_order_u - 3) * pow(corridor.scale_factor, 2 * min_order_u - 1);
                            idx ++ ;
                        }

            sub_shift += s1CtrlP_num;
        }
    } 

    QpGenSparseMa27 * qp 
    = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );

    QpGenData * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
        c,      irowQ,  nnzQ,   jcolQ,  dQ,
        xlow,   ixlow,  xupp,   ixupp,
        irowA,  nnzA,   jcolA,  dA,     b,
        irowC,  nnzC,   jcolC,  dC,
        clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars  = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid = (QpGenResiduals *) qp->makeResiduals( prob );
    GondzioSolver  * s     = new GondzioSolver( qp, prob );
    
    // Turn Off/On the print of the solving process
    // s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);

    if( ierr == 0 ) 
    {
        double d_var[nx];
        vars->x->copyIntoArray(d_var);

        PolyCoeff = MatrixXd::Zero(segment_num, s1CtrlP_num );
        PolyTime  = VectorXd::Zero(segment_num);
        obj = 0.0;
        
        int var_shift = 0;

        MatrixXd Q_o(s1d1CtrlP_num, s1d1CtrlP_num);
        int min_order_l = floor(minimize_order);
        int min_order_u = ceil (minimize_order);

        for(int i = 0; i < segment_num; i++ )
        {   
            PolyTime(i) = time[i];

            for(int j = 0; j < s1CtrlP_num; j++)
                PolyCoeff(i , j) = d_var[j + var_shift];

            var_shift += s1CtrlP_num;

            // Can't figure out how to dig out the objective value from ooqp solver, have to calculate it manually.
            double scale = time[i];
            if(min_order_l == min_order_u)
                Q_o = MQM_u / (double)pow(scale, 2 * min_order_u - 3) * pow(corridor.scale_factor, 2 * min_order_u - 1);
            else
                Q_o = MQM_l / (double)pow(scale, 2 * min_order_l - 3) * pow(corridor.scale_factor, 2 * min_order_l - 1)
                    + MQM_u / (double)pow(scale, 2 * min_order_u - 3) * pow(corridor.scale_factor, 2 * min_order_u - 1);
        
            for(int p = 0; p < 3; p ++ )
            {   
                VectorXd coeff = PolyCoeff.row(i).segment(p * s1d1CtrlP_num, s1d1CtrlP_num);

                obj += (coeff.transpose() * Q_o * coeff)(0);
            }            
        }   
    } 
    else if( ierr == 3)
        cout << "The program is provably infeasible, check the formulation.\n";
    else if (ierr == 4)
        cout << "The program is very slow in convergence, may have numerical issue.\n";
    else
        cout << "Solver numerical error.\n";
    
    return ierr;
}