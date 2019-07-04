#include <global_planner/temporal_optimizer.h>
using namespace std;    
using namespace Eigen;

static void MSKAPI printstr(void *handle, MSKCONST char str[])
{
  printf("%s",str);
}

void temporalTrajOptimizer::timeProfileGeneration( 
    const MatrixXd & polyCoeff, 
    VectorXd & time, 
    const double & maxVel, 
    const double & maxAcc, 
    const double & maxdAcc, 
    const double & d_s, 
    const double & rho)
{
    /* minimum time physical feasible trajectory time allocator. */
    /* objective is to generate motion as fast as possible within the physical limitaion (vel, acc and jerk). */
    _P          = polyCoeff;
    _T          = time;

    _seg_num    = polyCoeff.rows();
    _poly_num1D = polyCoeff.cols() / 3;
    double maxJer_s = maxdAcc * d_s;

    /*** ## Stacking bounds for all unknowns ## ***/ 
    vector< pair<MSKboundkeye, pair<double, double> > > var_bdk; 
    vector< pair<MSKboundkeye, pair<double, double> > > var_bdk_n; 

    int num_a[_seg_num], num_b[_seg_num], num_c[_seg_num], num_d[_seg_num];
    int num_t[_seg_num], num_e[_seg_num], num_f[_seg_num], num_g[_seg_num], num_h[_seg_num];
    int num_x[_seg_num];

    int num_a_n = 0, num_b_n = 0, num_c_n = 0, num_d_n = 0;
    int num_t_n = 0, num_e_n = 0, num_f_n = 0, num_g_n = 0, num_h_n = 0;
    int num_x_n = 0;

    int _equ_equal_num = 0;  // equality constraints b(i+1) - b(i) == 2 * a(i) * d_s;
    int _equ_slack_num = 0;  // map e,f,g,h to original variables

    int _equ_vel_conti_num   = _seg_num - 1;        // continuity constraints for continuous velocity, note only in one axis continuous means continuous in x,y,z
    int _inequ_acc_conti_num = 3 * (_seg_num - 1);  // continuity constraints for acceleration by enforcing jerk_max bound deviation constraints

    int _inequ_vel_num = 0;   // linear bounding constraints for velocity in x, y, z axis
    int _inequ_acc_num = 0;   // linear bounding constraints for acceleration
    int _inequ_jer_num = 0;   // linear bounding constraints for jerk

    vector<VectorXd> s_list;
    VectorXd k_list(_seg_num);
    for(int k = 0; k < _seg_num; k++)
    {
        double duration  = time(k);
        int K = ceil(duration / d_s);

        k_list[k] = K;
        VectorXd s_k(K + 1);
        for(int i = 0; i < K + 1; i++)
            s_k(i) = i * d_s;

        s_list.push_back(s_k);

        num_a[k] = K; 
        num_b[k] = K + 1; 
        num_c[k] = K + 1;
        num_d[k] = K;
        num_t[k] = K; 
        num_e[k] = K;
        num_f[k] = K + 1; 
        num_g[k] = K + 1;  
        num_h[k] = K + 1;

        num_x[k] = num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + num_f[k] + num_g[k] + num_h[k];
        // get constraints number
        _equ_equal_num += K;
        _equ_slack_num += K + 3 * (K + 1);

        _inequ_vel_num += 3 * (K + 1);
        _inequ_acc_num += 3 *  K;
        _inequ_jer_num += 3 * (K - 1);

        // stack all boundary value for variables in one segment
        var_bdk.clear();
        // for a
        for(int i = 0; i < num_a[k]; i ++ )
        {
            pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) ); 
            var_bdk.push_back(vb_x);
        }
        // for b
        for(int i = 0; i < num_b[k]; i ++ )
        {
            pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_RA, make_pair( 0.0, + MSK_INFINITY ) ); 
            var_bdk.push_back(vb_x);
        }

        // for c + d
        for(int i = 0; i < num_c[k] + num_d[k]; i ++ )
        {
            pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) ); 
            var_bdk.push_back(vb_x);
        }
        // for t
        for(int i = 0; i < num_t[k]; i ++ )
        {
            pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FX, make_pair( sqrt(2.0), sqrt(2.0) ) ); 
            var_bdk.push_back(vb_x);
        }

        // for e,f,g,h
        for(int i = 0; i < num_e[k] + num_f[k] + num_g[k] + num_h[k]; i ++ )
        {
            pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) ); 
            var_bdk.push_back(vb_x);
        }

        // put variables for a segment to a large container
        for(auto ptr:var_bdk)
            var_bdk_n.push_back(ptr);
    }
    
    // slacked objective t0
    {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) ); 
        var_bdk_n.push_back(vb_x);
    }

    // slack variable 1.0 in the rotated objective induced cone
    {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FX, make_pair( 1.0, 1.0 ) ); 
        var_bdk_n.push_back(vb_x);
    }

    // get total variables number for each category
    for(int k = 0; k < _seg_num; k++)
    {
        num_a_n += num_a[k]; 
        num_d_n += num_d[k];
        num_b_n += num_b[k]; 
        num_c_n += num_c[k];
        num_t_n += num_t[k]; 
        num_e_n += num_e[k];
        num_f_n += num_f[k]; 
        num_g_n += num_g[k];
        num_h_n += num_h[k];
    }
    num_x_n = num_a_n + num_b_n + num_c_n + num_d_n + num_t_n + num_e_n + num_f_n + num_g_n + num_h_n;

    int _var_num = num_x_n + 2;
    int _equ_con_num   = _equ_equal_num + _equ_vel_conti_num; 
    int _inequ_con_num = _inequ_acc_conti_num + _inequ_vel_num + _inequ_acc_num + _inequ_jer_num;
    int _con_num = _equ_con_num + _inequ_con_num + _equ_slack_num;

    vector< pair<MSKboundkeye, pair<double, double> > > con_bdk; 
    double x_var[_var_num];
    MSKrescodee  r; 
    double primalobj;

    for(int i = 0; i < _equ_con_num; i ++)
    {   
        pair<MSKboundkeye, pair<double, double> > cb_eq = make_pair( MSK_BK_FX, make_pair(  0.0, 0.0 ) ); 
        con_bdk.push_back(cb_eq);
    }

    for(int i = 0; i < _equ_slack_num; i ++)
    {   
        pair<MSKboundkeye, pair<double, double> > cb_eq;
        
        if( i < num_e_n + num_f_n )
            cb_eq = make_pair( MSK_BK_FX, make_pair( 0.0, 0.0 ) ); 
        else if( i < num_e_n + num_f_n + num_g_n)
            cb_eq = make_pair( MSK_BK_FX, make_pair(-1.0, -1.0 ) ); 
        else
            cb_eq = make_pair( MSK_BK_FX, make_pair( 1.0, 1.0 ) ); 

        con_bdk.push_back(cb_eq);
    }

    for(int i = 0; i < _inequ_acc_conti_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( - maxJer_s, + maxJer_s ) ); 
        con_bdk.push_back(cb_ie);   
    }

    for(int i = 0; i < _inequ_vel_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( 0.0, maxVel * maxVel ) ); 
        con_bdk.push_back(cb_ie);   
    }

    for(int i = 0; i < _inequ_acc_num; i++)
    {   
        double lo_bnd;
        double up_bnd;

        if(i < 3 )
        {
            lo_bnd = - 0.01;
            up_bnd = + 0.01;            
        }
        else if( i >= _inequ_acc_num - 3 )
        {
            lo_bnd = - 0.01;
            up_bnd = + 0.01;            
        }
        else
        {
            lo_bnd = - maxAcc;
            up_bnd = + maxAcc;
        }

        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( lo_bnd, up_bnd ) ); 
        con_bdk.push_back(cb_ie);   
    }

    for(int i = 0; i < _inequ_jer_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_RA, make_pair( - maxJer_s, + maxJer_s ) ); 
        con_bdk.push_back(cb_ie);   
    }

    MSKint32t  j,i; 
    MSKenv_t   env; 
    MSKtask_t  task; 

    r = MSK_makeenv( &env, NULL ); 
    r = MSK_maketask(env, _con_num, _var_num, &task); 

    // Parameters used in the optimizer
    MSK_putintparam (task, MSK_IPAR_OPTIMIZER , MSK_OPTIMIZER_CONIC );
    
    // Turn Off/On the print of the solving process
    //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr); 
    if ( r == MSK_RES_OK ) 
    {
        r = MSK_appendcons(task, _con_num);  
        r = MSK_appendvars(task, _var_num); 
    }

    // Stacking the variable bounds   
    for(j = 0; j<_var_num && r == MSK_RES_OK; ++j){       // Set the bounds on variable j : //  blx[j] <= x_j <= bux[j] 
        if (r == MSK_RES_OK) 
            r = MSK_putvarbound(task, 
                                j,                            // Index of variable. 
                                var_bdk_n[j].first,           // Bound key.
                                var_bdk_n[j].second.first,    // Numerical value of lower bound.
                                var_bdk_n[j].second.second ); // Numerical value of upper bound.      
    } 
    
    // Stack the constraints bounds   
    //   for i=1, ...,con_num : blc[i] <= constraint i <= buc[i] 
    assert(_con_num == (int)con_bdk.size());

    for( i = 0; i < _con_num && r == MSK_RES_OK; i++ ) 
    {
        r = MSK_putconbound(task, 
                            i,                            // Index of constraint. 
                            con_bdk[i].first,             // Bound key.
                            con_bdk[i].second.first,      // Numerical value of lower bound.
                            con_bdk[i].second.second );   // Numerical value of upper bound. 
    }


    int row_idx = 0;
    int idx_bias = 0;
    // Stack the equality constraints   
    // For equality constraints of mapping b_k to a_k   
    // b_k+1 - b_K - 2 * a_k * d_s == 0.0;
    // ==> - 2 * d_s * a_k - b_K + b_k+1 == 0.0;
    for(int k = 0; k < _seg_num; k ++) // mapping b to a for each segment of the trajectory
    {   
        int K = k_list[k];

        for(int i = 0; i < K; i++)
        {
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = -2.0 * d_s;
            aval[1] = -1.0;
            aval[2] =  1.0;

            asub[0] = idx_bias + i;
            asub[1] = idx_bias + num_a[k] + i;
            asub[2] = idx_bias + num_a[k] + i + 1;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }
    
    // For equality constraints of continuity at velocity 
    // continuous in b_k between each consecutive curve 
#if 1    
    idx_bias = num_x[0];
    for(int k = 1; k < _seg_num; k ++) 
    {   
        int nzi = 2;
        MSKint32t asub[nzi];
        double aval[nzi];
        aval[0] =  1.0;
        aval[1] = -1.0;

        asub[0] = idx_bias - num_x[k-1] + num_a[k-1] + num_b[k-1] - 1;
        asub[1] = idx_bias              + num_a[k];

        r = MSK_putarow(task, row_idx, nzi, asub, aval);    
        row_idx ++;
        idx_bias += num_x[k];
    }
#endif

    // For equality constraints of mapping c_k+1 + c_k to e_k 
    // e_k = c_k + c_k+1
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        for(int i = 0; i < K; i++ )
        {
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] =  1.0;
            aval[1] =  1.0;
            aval[2] = -1.0;

            asub[0] = idx_bias + num_a[k] + num_b[k] + i;
            asub[1] = idx_bias + num_a[k] + num_b[k] + i + 1;
            asub[2] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + i;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }

    // For equality constraints of mapping 2*c_k to f_k 
    // f_k = 2 * c_k
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        for(int i = 0; i < K + 1; i++ )
        {
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] =  2.0;
            aval[1] = -1.0;

            asub[0] = idx_bias + num_a[k] + num_b[k] + i;
            asub[1] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + i;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }

    // For equality constraints of mapping g_k to b_k - 1 
    // g_k = b_k - 1
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        for(int i = 0; i < K + 1; i++ )
        {
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = -1.0;
            aval[1] =  1.0;

            asub[0] = idx_bias + num_a[k] + i;
            asub[1] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + num_f[k] + i;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }

    // For equality constraints of mapping h_k to b_k + 1 
    // h_k = b_k + 1
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        for(int i = 0; i < K + 1; i++ )
        {
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = -1.0;
            aval[1] =  1.0;

            asub[0] = idx_bias + num_a[k] + i;
            asub[1] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + num_f[k] + num_g[k] + i;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }
    
    // For equality constraints of continuity at acceleration, by limiting piecewise deviation 
    // continuous in b_k between each consecutive curve 
#if 1
    idx_bias = num_x[0];
    for(int k = 1; k < _seg_num; k ++) 
    {   
        int K_f = k_list[k-1];
        VectorXd s_0 = s_list[k];
        VectorXd s_f = s_list[k-1];
        double s0 = (s_0(0)   + s_0(1))     / 2.0;
        double sf = (s_f(K_f) + s_f(K_f-1)) / 2.0;

        Vector3d f_0 = getVel( k,   s0 );
        Vector3d f_f = getVel( k-1, sf );

        Vector3d h_0 = getAcc( k,   s0 );
        Vector3d h_f = getAcc( k-1, sf );

        for(int i = 0; i < 3; i++) //for x, y, and z axis    
        {
            int nzi = 6;
            MSKint32t asub[nzi];
            double aval[nzi];
            aval[0] = f_f(i);
            aval[1] = h_f(i) / 2.0;
            aval[2] = h_f(i) / 2.0;

            aval[3] = -f_0(i);
            aval[4] = -h_0(i) / 2.0;
            aval[5] = -h_0(i) / 2.0;

            asub[0] = idx_bias - num_x[k-1] + num_a[k-1] - 1;
            asub[1] = idx_bias - num_x[k-1] + num_a[k-1] + num_b[k-1] - 2;
            asub[2] = idx_bias - num_x[k-1] + num_a[k-1] + num_b[k-1] - 1;

            asub[3] = idx_bias;
            asub[4] = idx_bias + num_a[k];
            asub[5] = idx_bias + num_a[k] + 1;

            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        idx_bias += num_x[k];
    }
#endif

    // For inequality constraints of constraining velocity within limit
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        VectorXd s = s_list[k];
        for(int p = 0; p < K + 1; p ++)
        {
            Vector3d f = getVel( k, s(p) );

            for(int i = 0; i < 3; i++) //for x, y, and z axis    
            {
                int nzi = 1;
                MSKint32t asub[nzi];
                double aval[nzi];

                aval[0] = pow(f(i), 2);                
                asub[0] = idx_bias + num_a[k] + p;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
        }
        idx_bias += num_x[k];
    }

    // For inequality constraints of constraining acceleration within limit 
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        VectorXd s = s_list[k];
        for(int p = 0; p < K; p ++)
        {   
            double s_a = (s(p) + s(p+1)) / 2.0;
            Vector3d f = getVel( k, s_a );
            Vector3d h = getAcc( k, s_a );
            
            for(int i = 0; i < 3; i++) //for x, y and z axis    
            {
                int nzi = 3;
                MSKint32t asub[nzi];
                double aval[nzi];

                aval[0] = f(i);
                aval[1] = h(i) / 2.0;
                aval[2] = h(i) / 2.0;

                asub[0] = idx_bias + p;
                asub[1] = idx_bias + num_a[k] + p;
                asub[2] = idx_bias + num_a[k] + p + 1;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
        }

        idx_bias += num_x[k];
    }

    // For inequality constraints of constraining jerk within limit
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k ++) 
    {   
        int K = k_list[k];
        VectorXd s = s_list[k];
        for(int p = 0; p < K - 1; p ++)
        {   
            double s_a_1 = (s(p)   + s(p+1)) / 2.0;
            double s_a_2 = (s(p+1) + s(p+2)) / 2.0;

            Vector3d f_1 = getVel( k, s_a_1 );
            Vector3d h_1 = getAcc( k, s_a_1 );

            Vector3d f_2 = getVel( k, s_a_2 );
            Vector3d h_2 = getAcc( k, s_a_2 );

            for(int i = 0; i < 3; i++) //for x, y and z axis    
            {
                int nzi = 5; 
                MSKint32t asub[nzi];
                double aval[nzi];

                aval[0] = f_1(i);
                aval[1] = h_1(i) / 2.0;
                aval[2] = h_1(i) / 2.0 - h_2(i) / 2.0;

                aval[3] = - f_2(i);
                aval[4] = - h_2(i) / 2.0;

                asub[0] = idx_bias + p;
                asub[1] = idx_bias + num_a[k] + p;
                asub[2] = idx_bias + num_a[k] + p + 1;

                asub[3] = idx_bias + p + 1;
                asub[4] = idx_bias + num_a[k] + p + 2;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
        }
        idx_bias += num_x[k];
    }

    // Stacking all conic cones
    idx_bias = 0;
    {   
        for(int k = 0; k < _seg_num; k++)
        {
            int K = k_list[k];
            // 2 * d_k * e_k >= t_k^2
            for(int i = 0; i < K; i++)
            {   
                int nzi = 3;
                MSKint32t csub[nzi];                
                
                csub[0] = idx_bias + num_a[k] + num_b[k] + num_c[k] + i;
                csub[1] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + i;

                csub[2] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + i;
                
                r = MSK_appendcone(task, MSK_CT_RQUAD, 0.0, nzi, csub);
            }        

            // h_k >= norm(f_k, g_k)
            for(int i = 0; i < K + 1; i++)
            {   
                int nzi = 3;
                MSKint32t csub[nzi];                
                
                csub[0] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + num_f[k] + num_g[k] + i;

                csub[1] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + i;
                csub[2] = idx_bias + num_a[k] + num_b[k] + num_c[k] + num_d[k] + num_t[k] + num_e[k] + num_f[k] + i;
                
                r = MSK_appendcone(task, MSK_CT_QUAD, 0.0, nzi, csub);
            }        

            idx_bias += num_x[k];
        }
    }

    // Stacking the quadratic induced rotated cone;
    {           
        int nzi = 2 + num_a_n;
        MSKint32t csub[nzi];                
        
        csub[0] = num_x_n;
        csub[1] = num_x_n + 1;

        int idx = 0;
        idx_bias = 0;
        
        for(int k = 0; k < _seg_num; k++)
        {
            for( int i = 0; i < num_a[k]; i++ )
            {
                csub[idx + 2] = idx_bias + i;
                idx ++;
            }

            idx_bias += num_x[k];
        }

        r = MSK_appendcone(task, MSK_CT_RQUAD, 0.0, nzi, csub);
    }

   // Stacking the objective function
    int nzi = num_d_n + 1;
    MSKint32t asub[nzi];
    double aval[nzi];

    aval[0] = rho * d_s;
    asub[0] = num_x_n;

    int idx = 1;
    
    idx_bias = 0;
    for(int k = 0; k < _seg_num; k++)
    {   
        int K = k_list[k];
        for(int i = 0; i < K; i++)
        {
            aval[idx] = 2.0 * d_s;
            asub[idx] = idx_bias + num_a[k] + num_b[k] + num_c[k] + i;
            idx ++;
        }
        idx_bias += num_x[k];
    }
    
    r = MSK_putclist(task, nzi, asub, aval);

    if ( r==MSK_RES_OK ) 
         r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
    
    if ( r==MSK_RES_OK ) 
    { 
        MSKrescodee trmcode; 
        r = MSK_optimizetrm(task,&trmcode); 
        MSK_solutionsummary (task,MSK_STREAM_LOG); 
          
        if ( r==MSK_RES_OK ) 
        { 
          MSKsolstae solsta; 
          MSK_getsolsta (task,MSK_SOL_ITR,&solsta); 
           
          switch(solsta) 
          { 
            case MSK_SOL_STA_OPTIMAL:    
            case MSK_SOL_STA_NEAR_OPTIMAL: 
              
            
            r = MSK_getxx(task, 
                          MSK_SOL_ITR,    // Request the interior solution.  
                          x_var); 

            r = MSK_getprimalobj(
                task,
                MSK_SOL_ITR,
                &primalobj);

            _objective = primalobj;
            
            break; 
            
            case MSK_SOL_STA_DUAL_INFEAS_CER: 
            case MSK_SOL_STA_PRIM_INFEAS_CER: 
            case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER: 
            case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:   
              printf("Primal or dual infeasibility certificate found.\n"); 
              break; 
               
            case MSK_SOL_STA_UNKNOWN: 
              printf("The status of the solution could not be determined.\n"); 
              break; 
            default: 
              printf("Other solution status."); 
              break; 
          } 
        } 
        else 
        { 
          printf("Error while optimizing.\n"); 
        } 
      }
     
      if (r != MSK_RES_OK) 
      { 
        // In case of an error print error code and description. 
        char symname[MSK_MAX_STR_LEN]; 
        char desc[MSK_MAX_STR_LEN]; 
         
        printf("An error occurred while optimizing.\n");      
        MSK_getcodedesc (r, 
                         symname, 
                         desc); 
        printf("Error %s - '%s'\n",symname,desc); 
      } 
    
    VectorXd sol(_var_num);
 
    vector<VectorXd> a_list, b_list, c_list, d_list;
    vector<VectorXd> t_list, e_list, f_list, g_list, h_list;

    for(int i = 0; i < _var_num; i++)
        sol(i) = x_var[i];

    double T = 0.0;
    double opt_duration;
    int num_x_k = 0;
    for(int k = 0; k < _seg_num; k++)
    {   
        opt_duration = 0.0;
        int K = k_list[k];
        VectorXd sol_k = sol.segment(num_x_k, num_x[k]);
        VectorXd b_k   = sol_k.segment(num_a[k], num_b[k]);

        for(int i = 0; i < K; i++)
        {
            if( b_k(i) <= 0.0 || b_k(i+1) <= 0.0 )
                continue;

            T            += 1.0 * 2 * d_s/(sqrt(b_k(i)) + sqrt(b_k(i+1)));
            opt_duration += 1.0 * 2 * d_s/(sqrt(b_k(i)) + sqrt(b_k(i+1)));
        }

        time(k) = opt_duration;
        num_x_k += num_x[k];
    }

    // Stacking the output results
    int max_K = -1;
    for(int i = 0; i < _seg_num; i++)
    {
        if(k_list(i) > max_K)
            max_K = k_list(i);
    }

    time_allocator = new timeAllocator(_seg_num, d_s, max_K, maxVel, maxAcc, maxJer_s);
    num_x_k = 0;
    for(int k = 0; k < _seg_num; k++)
    {   
        double T = 0.0;
        int K = k_list(k);

        time_allocator->K(k) = K;

        VectorXd sol_k = sol.segment(num_x_k, num_x[k]);
        VectorXd a_k   = sol_k.segment(0, num_a[k]);
        VectorXd b_k   = sol_k.segment(num_a[k], num_b[k]);

        for(int i = 0; i < K + 1; i++)
        {
            if(i <  K)
            {
                time_allocator->a(k, i) = a_k(i);

                if( b_k(i) <= 0.0 || b_k(i+1) <= 0.0 )
                    T += 0.0;
                else
                    T += 1.0 * 2 * d_s / (sqrt(b_k(i)) + sqrt(b_k(i+1)) );
                
                time_allocator->time(k, i) = T;
                
                if(i == 0)
                    time_allocator->time_acc(k, i) = time_allocator->time(k, i) / 2.0;
                else
                    time_allocator->time_acc(k, i) = (time_allocator->time(k, i) + time_allocator->time(k, i - 1)) / 2.0;
            }    
            
            time_allocator->b(k, i) = b_k(i);
            time_allocator->s(k, i) = s_list[k](i);
        }

        num_x_k += num_x[k];
    }

    MSK_deletetask(&task); 
    MSK_deleteenv(&env); 
}

Vector3d temporalTrajOptimizer::getVel(int k, double s)
{
    if(_type == 0)
        return getVelPoly(k ,s);
    else
        return getVelBezier(k ,s);
}

Vector3d temporalTrajOptimizer::getAcc(int k, double s)
{
    if(_type == 0)
        return getAccPoly(k ,s);
    else
        return getAccBezier(k ,s);
}

Vector3d temporalTrajOptimizer::getVelPoly(int k, double s)
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (_P.row(k) ).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd t = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
            if(j==0)
                t(j) = 0.0;
            else
                t(j) = j * pow(s, j-1);

        ret(dim) = coeff.dot(t);
    }

    return ret;
}

Vector3d temporalTrajOptimizer::getAccPoly(int k, double s)
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (_P.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd t = VectorXd::Zero( _poly_num1D );

        for(int j = 0; j < _poly_num1D; j ++)
            if( j==0 || j==1 )
                t(j) = 0.0;
            else
                t(j) = j * (j - 1) * pow(s, j-2);

        ret(dim) = coeff.dot(t);
    }

    return ret;
}

Vector3d temporalTrajOptimizer::getVelBezier(int k, double s)
{   
    s = s / _T(k);
    Vector3d ret = VectorXd::Zero(3);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < _ctrl_num1D; j++)
            if(j < _ctrl_num1D - 1 )
                ret(i) += _Cv(j) * _poly_order 
                              * ( _P(k, i * _ctrl_num1D + j + 1) - _P(k, i * _ctrl_num1D + j))
                              * pow(s, j) * pow((1 - s), (_poly_order - j - 1) ); 

    return ret;
}

Vector3d temporalTrajOptimizer::getAccBezier(int k, double s)
{   
    s = s / _T(k);
    Vector3d ret = VectorXd::Zero(3);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < _ctrl_num1D; j++)
            if(j < _ctrl_num1D - 2 )
              ret(i) += _Ca(j) * _poly_order * (_poly_order- 1) 
                        * ( _P(k, i * _ctrl_num1D + j + 2) - 2 * _P(k, i * _ctrl_num1D + j + 1) + _P(k, i * _ctrl_num1D + j))
                        * pow(s, j) * pow((1 - s), (_poly_order - j - 2) );                         

    return ret / _T(k);
}