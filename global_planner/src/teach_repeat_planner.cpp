#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/OptimalTimeAllocator.h>
#include <quadrotor_msgs/SpatialTemporalTrajectory.h>
#include <quadrotor_msgs/ReplanCheck.h>

#include <global_planner/utils/a_star.h>
#include <global_planner/utils/backward.hpp>
#include <global_planner/utils/poly_utils.h>
#include <global_planner/utils/bezier_base.h>
#include <global_planner/temporal_optimizer.h>
#include <global_planner/spatial_optimizer.h>

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _vis_traj_width;
double _x_size, _y_size, _z_size, _resolution, _inv_resolution;
double _cloud_margin, _minimize_order;
double _MAX_Vel, _MAX_Acc, _MAX_d_Acc;
int    _traj_order, _max_inf_iter, _max_clu_iter;
string _your_file_path;

// useful global variables
bool _has_odom  = false;
bool _has_map   = false;
bool _has_traj  = false;

Vector3d _start_pt,  _end_pt;
Vector3d _map_lower, _map_upper;

double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
double _rho;
int _max_x_id, _max_y_id, _max_z_id;
int _traj_id = 1;
int _vis_iter_num = 0;

vector<Vector3d> _manual_path;
decomp_ros_msgs::PolyhedronArray _poly_array_msg;

// ros related
ros::Subscriber _map_sub, _odom_sub, _joy_sub;
ros::Publisher _vis_polytope_pub, _vis_traj_pub, _vis_grid_path_pub, _vis_inf_map_pub;
ros::Publisher _space_time_pub;

ros::Time _odom_time, _traj_time_start, _traj_time_final, _time_update_odom;
nav_msgs::Odometry _odom;

// useful object
Bernstein      * _bezier_basis             = new Bernstein(); 
gridPathFinder * _path_finder              = new gridPathFinder();
polyhedronGenerator * _polyhedronGenerator = new polyhedronGenerator();

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void rcvOdometryCallBack(const nav_msgs::Odometry odom);
void rcvJoyCallBack(const sensor_msgs::Joy joy);

void trajPlanning();
void initTimeAllocation(decomp_cvx_space::FlightCorridor & corridor);
bool isNext(Vector3d coord_1, Vector3d coord_2);
double getEnergy(timeAllocator * time_allocator, const MatrixXd bezier_coeff, const VectorXd time);

void visCorridor( decomp_ros_msgs::PolyhedronArray & poly_array_msg );
void visGridPath();
void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time, int iter_id);
void visFinalBezierTrajectory(MatrixXd polyCoeff, VectorXd time);
void clearVisualization(int grid_path_mk_num, int traj_mk_iter_num);

quadrotor_msgs::SpatialTemporalTrajectory getSpaceTimeTraj(const timeAllocator * time_allocator, const MatrixXd & bezier_coeff, const VectorXd & range );

fstream path_record;
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{    
    _end_pt << wp.poses[0].pose.position.x,
               wp.poses[0].pose.position.y,
               wp.poses[0].pose.position.z;
}

bool _WRITE_PATH, _READ_PATH;
void rcvJoyCallBack(const sensor_msgs::Joy joy)
{   
    if(joy.buttons[7] == 1.0)
    {
        ROS_WARN("[trr_global_planner] Enter in autonomous mode");
        if (_WRITE_PATH)
        {
            for(auto pt: _manual_path)
                path_record<<pt(0)<<" "<<pt(1)<<" "<<pt(2)<<"\n";
        }
                
        if (_READ_PATH)
        {
            std::ifstream infile(_your_file_path);
            double x, y, z;

            while (infile >> x >> y >> z){
                Vector3d pt(x, y, z);
                _manual_path.push_back(pt);
            }

            _polyhedronGenerator->corridorIncreGeneration(_manual_path, _poly_array_msg);
        }

        if(_manual_path.size() == 0) return;

        Vector3d current_pt(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
        _path_finder->AstarSearch(current_pt, _manual_path.front());
        vector<Vector3d> connect_path = _path_finder->getPath();
        _path_finder->resetMap();

        // add polytopoes at newly found path to connect to starting position
        if( _polyhedronGenerator->corridorInsertGeneration(connect_path, _poly_array_msg) == 1 )
        {
            ROS_WARN("[trr_global_planner] Add extra polytopes to connecting path");
            visCorridor(_poly_array_msg);
        }
        
        for(auto coord: _manual_path) connect_path.push_back(coord);
        _manual_path = connect_path;
        visGridPath( );

        trajPlanning(); 
    }
    else if(joy.buttons[6] == 1.0)
    {
        ROS_WARN("[trr_global_planner] Enter in maunal flight mode");
        clearVisualization(_manual_path.size(), _vis_iter_num);

        _manual_path.clear();
        _polyhedronGenerator->reset();
        _has_traj = false;
        _vis_iter_num = 0;
    }
}

Vector3i _pose_idx, _pose_idx_lst;
void rcvOdometryCallBack(const nav_msgs::Odometry odom)
{   
    if(!_has_map)
        return;

//    cout<<"odom"<<endl;
    _odom = odom;
    _odom_time = odom.header.stamp;
    _time_update_odom = ros::Time::now();

    _has_odom = true;

    Vector3d pos, pos_round;
    pos(0)  = odom.pose.pose.position.x;
    pos(1)  = odom.pose.pose.position.y;
    pos(2)  = odom.pose.pose.position.z;    

    _pose_idx = _path_finder->coord2gridIndex(pos);
    vector<Vector3d> coord_set;

    if(_manual_path.size() == 0)
    {   
        pos_round = _path_finder->gridIndex2coord(_pose_idx);
        _manual_path.push_back(pos_round);
        _pose_idx_lst = _pose_idx;

        coord_set.push_back(pos_round);
    }
    else if( _pose_idx != _pose_idx_lst ) 
    {   
        if( _path_finder->IndexQuery(_pose_idx) > 0 ) 
            return;
        
        pos_round = _path_finder->gridIndex2coord(_pose_idx);
        
        if(isNext(_manual_path.back(), pos_round) == false)
        {
            _path_finder->AstarSearch(_manual_path.back(), pos_round);
            
            vector<Vector3d> localPath = _path_finder->getPath();

            for(auto coord: localPath)
            {   
                coord_set.   push_back(coord);
                _manual_path.push_back(coord);
            }

            _path_finder->resetMap();
        }
        else
        {
            coord_set.   push_back(pos_round);
            _manual_path.push_back(pos_round);
        }    

        _pose_idx_lst = _pose_idx;
    }
    else
        return;

    if( _has_traj || _READ_PATH) return;

    if( _polyhedronGenerator->corridorIncreGeneration(coord_set, _poly_array_msg) == 1 )
        visCorridor(_poly_array_msg);
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if( _has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_inf;
    pcl::fromROSMsg(pointcloud_map, cloud);
    sensor_msgs::PointCloud2 map_inflation;

    if( (int)cloud.points.size() == 0)
        return;

    pcl::PointXYZ pt, pt_inf;

    int inf_step   = round(_cloud_margin * _inv_resolution);
    int inf_step_z = max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;

                    Vector3d vec_inf(inf_x, inf_y, inf_z);
                    Vector3i idx_inf = _path_finder->coord2gridIndex(vec_inf);

                    // set in obstacle points
                    _path_finder->setObs(inf_x, inf_y, inf_z);
                    _polyhedronGenerator->setObs(idx_inf);

                    // rounding for visualizing the grid map
                    Vector3d coor_round = _path_finder->gridIndex2coord( idx_inf );
                    pt_inf.x = coor_round(0);
                    pt_inf.y = coor_round(1);
                    pt_inf.z = coor_round(2);
                    cloud_inf.points.push_back(pt_inf);
                }
            }
        }
    }

    _polyhedronGenerator->finishMap();

    cloud_inf.width    = cloud_inf.points.size();
    cloud_inf.height   = 1;
    cloud_inf.is_dense = true;

    pcl::toROSMsg(cloud_inf, map_inflation);
    map_inflation.header.frame_id = "/map";
    _vis_inf_map_pub.publish(map_inflation);

    _has_map = true;
}

void initTimeAllocation(decomp_cvx_space::FlightCorridor & corridor)
{   
    VectorXd time;
    time.resize((int)corridor.polyhedrons.size());

    vector<Vector3d> points;
    points.push_back (_start_pt);

    for(int i = 1; i < (int)corridor.polyhedrons.size(); i++)
        points.push_back(corridor.polyhedrons[i].seed_coord);

    points.push_back (_end_pt);

    double _Vel = _MAX_Vel;
    double _Acc = _MAX_Acc;

    for (int k = 0; k < (int)points.size() - 1; k++)
    {
        double dtxyz;
        Vector3d p0   = points[k];        
        Vector3d p1   = points[k + 1];    
        Vector3d d    = p1 - p0;          
        Vector3d v0(0.0, 0.0, 0.0);       
        double D    = d.norm();                  
        double V0   = v0.dot(d / D);             
        double aV0  = fabs(V0);           

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1);
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
        double dcct = _Vel / _Acc;                                              
        double dccd = _Acc * dcct * dcct / 2;                                   

        if (D < aV0 * aV0 / (2 * _Acc))
        {               
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
        }
        else if (D < accd + dccd)
        {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
        }
        else
        {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
        }

        time(k) = dtxyz; 
        corridor.appendTime( time(k) );   
    }
}

bool isNext(Vector3d coord_1, Vector3d coord_2)
{
    Vector3i index_1 = _path_finder->coord2gridIndex(coord_1);
    Vector3i index_2 = _path_finder->coord2gridIndex(coord_2);

    if( abs(index_1(0) - index_2(0)) <= 1 
     && abs(index_1(1) - index_2(1)) <= 1 
     && abs(index_1(2) - index_2(2)) <= 1 )
        return true;

    return false;
}

double getEnergy(timeAllocator * time_allocator, const MatrixXd bezier_coeff, const VectorXd time)
{
    double jerk_square_inte = 0;
    int traj_seg_num = bezier_coeff.rows();

    for(int i = 0; i < traj_seg_num; i++)
    {
        double jerk_square_inte_i = 0;
        int K = time_allocator->K(i);

        for(int k = 0; k < K; k++)
        {   
            double delta_t; 
            double s_k, s_k_1, a_k, b_k, b_k_1;
            Vector3d velocity_s, acceleration_s, acceleration, acceleration_1, acceleration_2, jerk;

            if( k == 0 )
            {   
                if(i == 0)
                {
                    delta_t = time_allocator->time_acc(i, k);
                    s_k     = time_allocator->s(i, k);
                    s_k_1   = time_allocator->s(i, k + 1);
                    a_k     = time_allocator->a(i, k);
                    b_k     = time_allocator->b(i, k);
                    b_k_1   = time_allocator->b(i, k + 1);

                    velocity_s     = _bezier_basis->getVel(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i));
                    acceleration_s = _bezier_basis->getAcc(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i)) /time(i);
                    acceleration   = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                    jerk = acceleration / delta_t;

                    for(int p = 0; p < 3; p++)
                    {   
                        jerk_square_inte_i += pow(jerk(p),2) * delta_t;
                    }
                }
                else // do nothing
                {
                    jerk_square_inte_i += 0.0;
                }

            }
            else if (k == K-1)
            {   
                if(i == traj_seg_num - 1)
                {
                    delta_t = time_allocator->time(i, k) - time_allocator->time_acc(i, k);
                    s_k     = time_allocator->s(i, k);
                    s_k_1   = time_allocator->s(i, k + 1);
                    a_k     = time_allocator->a(i, k);
                    b_k     = time_allocator->b(i, k);
                    b_k_1   = time_allocator->b(i, k + 1);

                    velocity_s     = _bezier_basis->getVel(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i));
                    acceleration_s = _bezier_basis->getAcc(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i)) /time(i);
                    acceleration   = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                    jerk = acceleration / delta_t;
                    //cout<<"case 3"<<endl;
                }
                else
                {
                    delta_t = time_allocator->time(i, k) - time_allocator->time_acc(i, k) + time_allocator->time_acc(i + 1, 0);
                    s_k     = time_allocator->s(i, k);
                    s_k_1   = time_allocator->s(i, k + 1);
                    a_k     = time_allocator->a(i, k);
                    b_k     = time_allocator->b(i, k);
                    b_k_1   = time_allocator->b(i, k + 1);

                    velocity_s     = _bezier_basis->getVel(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i));
                    acceleration_s = _bezier_basis->getAcc(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i)) /time(i);
                    acceleration_1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                    s_k     = time_allocator->s(i + 1, 0);
                    s_k_1   = time_allocator->s(i + 1, 1);
                    a_k     = time_allocator->a(i + 1, 0);
                    b_k     = time_allocator->b(i + 1, 0);
                    b_k_1   = time_allocator->b(i + 1, 1);

                    velocity_s     = _bezier_basis->getVel(bezier_coeff, i + 1, (s_k + s_k_1 ) / 2.0 /time(i + 1));
                    acceleration_s = _bezier_basis->getAcc(bezier_coeff, i + 1, (s_k + s_k_1 ) / 2.0 /time(i + 1)) /time(i + 1);
                    acceleration_2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                    jerk = (acceleration_2 - acceleration_1) / delta_t;
                }

                for(int p = 0; p < 3; p++)
                {   
                    jerk_square_inte_i += pow(jerk(p),2) * delta_t;
                }
            }
            else
            {
                delta_t = time_allocator->time_acc(i, k + 1) - time_allocator->time_acc(i, k);

                s_k     = time_allocator->s(i, k);
                s_k_1   = time_allocator->s(i, k + 1);
                a_k     = time_allocator->a(i, k);
                b_k     = time_allocator->b(i, k);
                b_k_1   = time_allocator->b(i, k + 1);

                velocity_s     = _bezier_basis->getVel(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i));
                acceleration_s = _bezier_basis->getAcc(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i)) /time(i);
                acceleration_1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k     = time_allocator->s(i, k + 1);
                s_k_1   = time_allocator->s(i, k + 2);
                a_k     = time_allocator->a(i, k + 1);
                b_k     = time_allocator->b(i, k + 1);
                b_k_1   = time_allocator->b(i, k + 2);

                velocity_s     = _bezier_basis->getVel(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i));
                acceleration_s = _bezier_basis->getAcc(bezier_coeff, i, (s_k + s_k_1 ) / 2.0 /time(i)) /time(i);
                acceleration_2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                jerk = (acceleration_2 - acceleration_1) / delta_t;
                for(int p = 0; p < 3; p++)
                {   
                    jerk_square_inte_i += pow(jerk(p),2) * delta_t;
                }
            }
        }

        jerk_square_inte += jerk_square_inte_i;
    }

    return jerk_square_inte;
}

void UpdateTime(decomp_cvx_space::FlightCorridor & corridor, VectorXd bezier_time)
{
    corridor.durations.clear();
    for(int i = 0; i < (int)bezier_time.size(); i++)
        corridor.appendTime(bezier_time(i));
}   

void trajPlanning()
{   
    if( _has_map == false || _has_odom == false) 
        return;

    _start_pt = _manual_path.front();
    _end_pt   = _manual_path.back();

    decomp_cvx_space::FlightCorridor corridor = _polyhedronGenerator->getCorridor();

    ROS_WARN("[trr_global_planner] Start coordinate descent-based spatial-temporal joint optimization");
    MatrixXd bezier_coeff;
    VectorXd bezier_time;
    
    /*  Initial time allocation  */
    initTimeAllocation(corridor);

    MatrixXd pos = MatrixXd::Zero(2,3);
    MatrixXd vel = MatrixXd::Zero(2,3);
    MatrixXd acc = MatrixXd::Zero(2,3);

    pos.row(0) = _start_pt;
    pos.row(1) = _end_pt;    

    vel.row(0) << 0.0, 0.0, 0.0;
    acc.row(0) << 0.0, 0.0, 0.0;

    double delta_s = 0.025;
    int iter_num = 0;
    int iter_max = 50;
    double J, J_lst, J_best;
    J_lst = inf;

    temporalTrajOptimizer * time_optimizer  = new temporalTrajOptimizer();
    spatialTrajOptimizer  * curve_optimizer = new spatialTrajOptimizer();
    timeAllocator         * time_profile_   = new timeAllocator();

    time_optimizer->setParam(_traj_order, _bezier_basis->getC(), _bezier_basis->getC_v(), _bezier_basis->getC_a());
    time_optimizer->setType(1);

    VectorXd bezier_range_;
    MatrixXd bezier_coeff_;

    MatrixXd Qo_u = _bezier_basis->getMQM_u();
    MatrixXd Qo_l = _bezier_basis->getMQM_l();
    while(iter_num < iter_max)
    {
        ros::Time time_before_optimization = ros::Time::now();
        
        int error_code = 
        curve_optimizer->bezierCurveGeneration(
                corridor, Qo_u, Qo_l, pos, vel, acc, _traj_order, _minimize_order, _MAX_Vel, _MAX_Acc);

        ros::Time time_after_optimization = ros::Time::now();
        ROS_WARN("[trr_global_planner] Time consumation of spatial optimization is: %f",(time_after_optimization - time_before_optimization).toSec() );

        bezier_coeff = curve_optimizer->getPolyCoeff();
        bezier_time  = curve_optimizer->getPolyTime();

        VectorXd bezier_range = bezier_time;

        if( error_code != 0 ){
            ROS_WARN("[trr_global_planner] Cannot find a safe solution, somthing wrong with the solver");  
            return;
        }
        else{   
            time_optimizer->timeProfileGeneration( bezier_coeff, bezier_time, _MAX_Vel, _MAX_Acc, _MAX_d_Acc, delta_s, 0.0 ); // 0.0 for minimizing the time, no regulizer in control cost
            timeAllocator * time_profile = time_optimizer->getTimeProfile();
        
            double energy = getEnergy(time_profile, bezier_coeff, bezier_range);
            J = energy + _rho * bezier_time.sum();
            
            cout<<"Energy cost is: "<<energy<<", Time duration is: "<<bezier_time.sum()<<endl;
            if((pow((J - J_lst), 2) < J_lst * 0.01) || J > J_lst)
                break;
            else{
                J_lst  = J;
                J_best = J;
                bezier_coeff_ = bezier_coeff;
                bezier_range_ = bezier_range;
                time_profile_ = time_profile;
            }
            
            visBezierTrajectory(bezier_coeff, bezier_range, iter_num);
            UpdateTime(corridor, bezier_time);
        }

        iter_num ++;
    }

    ROS_WARN("[trr_global_planner] find a solution after iteration: %d, the optimal cost is: %f", iter_num, J_best);

    visFinalBezierTrajectory(bezier_coeff_, bezier_range_); 

    _traj_time_start = _odom_time + ( ros::Time::now() - _time_update_odom);
    _traj_time_final = _traj_time_start;

    for(int i = 0; i < time_profile_->time.rows(); i++){   
        int K = time_profile_->K(i);
        _traj_time_final += ros::Duration(time_profile_->time(i, K - 1));
    }

    quadrotor_msgs::SpatialTemporalTrajectory space_time_msgs 
        = getSpaceTimeTraj(time_profile_, bezier_coeff_, bezier_range_);

    _space_time_pub.publish(space_time_msgs);

    _traj_id ++;
    _has_traj = true;
    
    cout<<"publish global planning"<<endl;

    delete curve_optimizer;
    delete time_optimizer;
    delete time_profile_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trr_global_planner_node");
    ros::NodeHandle nh("~");

    nh.param("write_path", _WRITE_PATH, false);
    nh.param("read_path",  _READ_PATH,  false);

    nh.param("map/map_margin", _cloud_margin, 0.25);
    nh.param("map/resolution", _resolution,   0.2 );
    
    nh.param("map/x_size",         _x_size, 50.0);
    nh.param("map/y_size",         _y_size, 50.0);
    nh.param("map/z_size",         _z_size, 5.0 );
    
    nh.param("planning/rho_time",  _rho,       1.0);
    nh.param("planning/max_vel",   _MAX_Vel,   1.0);
    nh.param("planning/max_acc",   _MAX_Acc,   1.0);
    nh.param("planning/max_d_acc", _MAX_d_Acc, 1.0);

    nh.param("planning/max_inf_iter", _max_inf_iter,        10  );
    nh.param("planning/max_clu_iter", _max_clu_iter,        50  );

    nh.param("optimization/min_order",  _minimize_order,    3.0 );
    nh.param("optimization/poly_order", _traj_order,        10  );

    nh.param("vis/vis_traj_width",     _vis_traj_width,     0.15);

    nh.param("your_file_path",         _your_file_path,     string("")  );

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _odom_sub = nh.subscribe( "odometry",  1, rcvOdometryCallBack);
    _joy_sub  = nh.subscribe( "joystick",  1, rcvJoyCallBack );

    // for visualization of the planning results
    _vis_traj_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);    
    _vis_grid_path_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
    _vis_inf_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("inflation_map", 10);
    _vis_polytope_pub  = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_corridor_mesh", 1, true);
    _space_time_pub    = nh.advertise<quadrotor_msgs::SpatialTemporalTrajectory>("space_time_traj", 10); 

    _map_lower << -_x_size/2.0, -_y_size/2.0, 0.0;
    _map_upper << +_x_size/2.0, +_y_size/2.0, _z_size;

    _poly_array_msg.header.frame_id = "/map";
    _vis_polytope_pub.publish(_poly_array_msg); 

    _pt_max_x = + _x_size / 2.0; _pt_min_x = - _x_size / 2.0;
    _pt_max_y = + _y_size / 2.0; _pt_min_y = - _y_size / 2.0; 
    _pt_max_z = + _z_size;       _pt_min_z = 0.0;
    
    _resolution = max(0.1, _resolution); // In case a too fine resolution crashes the CUDA code.
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _bezier_basis = new Bernstein(_minimize_order);
    _bezier_basis ->setFixedOrder(_traj_order);

    _path_finder  = new gridPathFinder(_max_x_id, _max_y_id, _max_z_id);
    _path_finder  -> initGridMap(_resolution, _map_lower, _map_upper);
    
    _polyhedronGenerator->initialize(false, true, true, // method in generating corridor
        _max_x_id, _max_y_id, _max_z_id, _map_lower, _map_upper, _resolution, _inv_resolution, // map information
        _max_inf_iter, _max_clu_iter); // max infaltion/clustering num

    if(_WRITE_PATH){
        path_record.open(_your_file_path);
    }

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status){
        ros::spinOnce();  
        status = ros::ok();
        rate.sleep();
    }

    delete _bezier_basis;
    delete _path_finder;

    return 0;
}

quadrotor_msgs::SpatialTemporalTrajectory getSpaceTimeTraj(const timeAllocator * time_allocator, const MatrixXd & bezier_coeff, const VectorXd & range )
{
    quadrotor_msgs::SpatialTemporalTrajectory space_time_traj;
    space_time_traj.action = quadrotor_msgs::OptimalTimeAllocator::ACTION_ADD;
    space_time_traj.header.frame_id = "/trr_global_planner";
    space_time_traj.trajectory_id = _traj_id;

    space_time_traj.header.stamp = _traj_time_start; 
    space_time_traj.start_time   = _traj_time_start; 
    space_time_traj.final_time   = _traj_time_final; 

    int seg_num = time_allocator->seg_num;
    space_time_traj.s_step = time_allocator->s_step; 
    space_time_traj.K_max  = time_allocator->K_max;

    for (int i = 0; i < seg_num; i++ )
    {     
        space_time_traj.K.push_back(time_allocator->K(i));

        for(int j = 0; j < time_allocator->K(i) + 1; j++ )
        {   
            if( j < time_allocator->K(i) )
            {
                space_time_traj.a.push_back( time_allocator->a(i, j) );
                space_time_traj.time.push_back    ( time_allocator->time(i, j) );
                space_time_traj.time_acc.push_back( time_allocator->time_acc(i, j) );
            }

            space_time_traj.b.push_back( time_allocator->b(i, j) );
            space_time_traj.s.push_back( time_allocator->s(i, j) );
        }
    }

    /*  stack the spatial curve  */
    seg_num = range.size();
    space_time_traj.num_segment = seg_num;

    for(int i = 0; i < seg_num; i++ )
    {    
        int poly_num1d = _traj_order + 1;
        for(int j =0; j < poly_num1d; j++)
        { 
            space_time_traj.coef_x.push_back(bezier_coeff(i,                  j));
            space_time_traj.coef_y.push_back(bezier_coeff(i,     poly_num1d + j));
            space_time_traj.coef_z.push_back(bezier_coeff(i, 2 * poly_num1d + j));
        }
        space_time_traj.range.push_back(range(i));
    }

    space_time_traj.start_yaw = 0.0;
    space_time_traj.final_yaw = 0.0;
    space_time_traj.trajectory_id = _traj_id;

    return space_time_traj;
}

geometry_msgs::Point Vector2Point(Vector3d vec)
{
    geometry_msgs::Point pt;
    pt.x = vec(0);   
    pt.y = vec(1);   
    pt.z = vec(2);   

    return pt;
}

void clearVisualization(int grid_path_mk_num, int traj_mk_iter_num)
{
    // 1. Clear the Grid Path :
    visualization_msgs::MarkerArray grid_vis; 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "trr_global_planner/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::DELETE;
    
    for(int i = 0; i < grid_path_mk_num; i++)
    {
        mk.id = i;
        grid_vis.markers.push_back(mk);
    }

    _vis_grid_path_pub.publish(grid_vis);

    // 2. Clear the Bezier Curves
    cout<<"bezier curves visualization num: "<<_vis_iter_num<<endl;
    for(int i = 0; i < _vis_iter_num; i++)
    {
        visualization_msgs::Marker traj_vis;

        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "map";
        traj_vis.ns = "trr_global_planner/trajectory" + to_string(i);
        traj_vis.id = 0;
        traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_vis.action = visualization_msgs::Marker::DELETE;

        for(int k = 0; k < 100; k++)
            _vis_traj_pub.publish(traj_vis);
    }

    // 3. Clear the polyhedrons
    _poly_array_msg.polyhedrons.clear();
    _poly_array_msg.ids.clear();
    _vis_polytope_pub.publish(_poly_array_msg);
}

void visCorridor( decomp_ros_msgs::PolyhedronArray & poly_array_msg )
{
    _vis_polytope_pub.publish(poly_array_msg);
}

void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time, int iter_id)
{   
    visualization_msgs::Marker traj_vis;

    _vis_iter_num ++;
    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory" + to_string(iter_id);
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = min(1.0, (iter_id + 1) / 5.0);
    traj_vis.color.g = 0.0;//max(0.0, 1 - rgb / 5.0);
    traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = polyCoeff.rows();
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
            state = _bezier_basis->getPosFromBezier( polyCoeff, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    _vis_traj_pub.publish(traj_vis);
}

void visFinalBezierTrajectory(MatrixXd polyCoeff, VectorXd time)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_final";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = polyCoeff.rows();
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
            state = _bezier_basis->getPosFromBezier( polyCoeff, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    _vis_traj_pub.publish(traj_vis);
}

void visGridPath( )
{   
    visualization_msgs::MarkerArray grid_vis; 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "trr_global_planner/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    
    int idx = 0;
    for(int i = 0; i < int(_manual_path.size()); i++)
    {
        mk.id = idx;
        mk.pose.position.x = _manual_path[i](0); 
        mk.pose.position.y = _manual_path[i](1); 
        mk.pose.position.z = _manual_path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        grid_vis.markers.push_back(mk);
    }

    _vis_grid_path_pub.publish(grid_vis);
}