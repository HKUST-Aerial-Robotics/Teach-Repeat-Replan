#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <time.h>
#include <random>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/Bspline.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/ReplanCheck.h>
#include <quadrotor_msgs/SpatialTemporalTrajectory.h>

#include <grad_replanner/backward.hpp>
#include <grad_replanner/bezier_base.h>

#include "grad_replanner/grad_band_optimizer.h"
#include "grad_replanner/non_uniform_bspline.h"
#include "grad_replanner/bspline_replanner.h"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

#define INIT_TRAJ_ID 0
#define OPT_TRAJ_ID 1
#define INIT_PT_ID 2
#define OPT_PT_ID 3

// ros related
ros::Subscriber _mission_sub;

ros::Publisher  _bspline_pub, _emergy_traj_pub, _grid_mk_pub;
ros::Publisher  _vis_check_traj_pub, _vis_initial_b_spline, _vis_opt_traj_pub;
ros::Publisher  _vis_vel_pub, _vis_acc_pub, _vis_fill_esdf_pub;
ros::Timer      _vis_esdf_grid_timer;

// simulation param from launch file
double _MAX_Vel, _MAX_Acc;
double _time_emergency;
double _esdf_grid_freq, _vis_traj_width;
string _frame_id;

// useful global variables
bool _has_odom   = false;
bool _on_replan  = false;

double _replan_traj_time, _time_extra;
double _t_bspline_cmd_start, _t_bspline_cmd_end;

ros::Time _replan_final_time = ros::TIME_MIN;
ros::Time _replan_start_time = ros::TIME_MAX;

int _replan_traj_id = 1;
shared_ptr<SDFMap> _sdf_map;
unique_ptr<BsplineReplanner> _bspline_replanner;
int succ_replan_num_ = 0;

double _traj_resolution;
vector<Vector3d> _traj_pts;
void gradLocalReplan(const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc, double local_to_global_time);
void BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt);

void displayPathWithColor(vector<Eigen::Vector3d> path, Eigen::Vector4d color, int id);
void drawBspline(NonUniformBspline bspline, int id, Eigen::Vector4d color, bool show_ctrl_pts,
                 double size2, int id2, Eigen::Vector4d color2);

void visCheckTrajPoints(const vector<Vector3d> & traj_pts);
void visBsplineInitialization(const MatrixXd & ctrl_pts);
void visFillMinima(Eigen::Vector3d center, Eigen::Vector3d sc);

ros::Time _replan_request_time;
double _replan_time_length;

bool is_first_request = false;
int request_cnt = 0;
ros::Time time_1st_request;
void rcvReplanRequestCallBack( const quadrotor_msgs::ReplanCheck & replan_request_msg)
{   
    if( !_sdf_map->hasDepthObservation() ) 
        return;
    
    if(request_cnt == 0)
        time_1st_request = ros::Time::now();

    request_cnt ++;

    ros::Time t0 = ros::Time::now();
    _replan_request_time = replan_request_msg.header.stamp;
    _replan_time_length  = replan_request_msg.replan_time_length;

    double plan_time_interval  = replan_request_msg.plan_points_time_interval;
    double check_time_interval = replan_request_msg.check_points_time_interval;
    
    MatrixXd acc(2, 3), vel(2, 3);

    Vector3d start_vel(replan_request_msg.start_velocity.x,     replan_request_msg.start_velocity.y,     replan_request_msg.start_velocity.z);
    Vector3d target_vel(replan_request_msg.stop_velocity.x,     replan_request_msg.stop_velocity.y,      replan_request_msg.stop_velocity.z);

    Vector3d start_acc(replan_request_msg.start_acceleration.x, replan_request_msg.start_acceleration.y, replan_request_msg.start_acceleration.z);
    Vector3d target_acc(replan_request_msg.stop_acceleration.x, replan_request_msg.stop_acceleration.y,  replan_request_msg.stop_acceleration.z);

    vel.row(0) = start_vel;
    vel.row(1) = target_vel;

    acc.row(0) = start_acc;
    acc.row(1) = target_acc;

    Vector3d check_pt;

    std::vector<Vector3d> traj_pts;            
    for(auto pt: replan_request_msg.plan_points){   
        Vector3d plan_pt(pt.x, pt.y, pt.z);
        traj_pts.push_back(plan_pt);    
    }
    visCheckTrajPoints(traj_pts);
	
    ros::Time t1 = ros::Time::now();
    //cout<<"time vis: ="<<(t1-t0).toSec()<<endl;
    Eigen::Vector3d grad;
    Eigen::Vector3d pt_1(replan_request_msg.check_points[0].x,replan_request_msg.check_points[0].y,replan_request_msg.check_points[0].z);
    for(int i = 0; i < replan_request_msg.check_points.size(); i++)
    {   
        check_pt << replan_request_msg.check_points[i].x, replan_request_msg.check_points[i].y, replan_request_msg.check_points[i].z;
    
        if( (check_pt-pt_1).norm() > 3.0 ) break;	
	   
	    double dis;
        dis = _sdf_map->getDistWithGradTrilinear(check_pt,  grad );
        if( dis <= 0.05  ){	   
            ROS_WARN("[Local Replanner] Collision detected, neeed replanning, dis:= %f", dis ); 
            if( i * check_time_interval < _time_emergency){
                ROS_ERROR("[Local Replanner] Emergency braking");
                
                quadrotor_msgs::Bspline stop_traj;
                stop_traj.start_time = _replan_request_time;
                stop_traj.traj_id    = _replan_traj_id ++;
                stop_traj.action = quadrotor_msgs::Bspline::ACTION_WARN_IMPOSSIBLE;
                _bspline_pub.publish(stop_traj);
                return;
            }
            else{   
                ros::Time t1 = ros::Time::now();
                gradLocalReplan(traj_pts, plan_time_interval, vel, acc, replan_request_msg.replan_to_global_time);
                //cout<<"time in local replannning := "<<(ros::Time::now() - t1).toSec()<<endl;
                return;
            }
        }
    }
}

quadrotor_msgs::Bspline getBsplineTraj(NonUniformBspline & traj_opt)
{   
    quadrotor_msgs::Bspline bspline;
    bspline.order = 3;

    bspline.start_time = _replan_request_time;
    bspline.traj_id    = _replan_traj_id ++;

    Eigen::MatrixXd ctrl_pts = traj_opt.getControlPoint();
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
        Eigen::Vector3d pvt = ctrl_pts.row(i);
        geometry_msgs::Point pt;
        pt.x = pvt(0);
        pt.y = pvt(1);
        pt.z = pvt(2);
        bspline.pts.push_back(pt);
    }
    Eigen::VectorXd knots = traj_opt.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
    }

    return bspline;
}

void gradLocalReplan( const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc , double local_to_global_time)
{
    /* generate sample points set on a naive trajectory*/
    vector<Eigen::Vector3d> start_end_derivative;
    start_end_derivative.push_back(vel.row(0));
    start_end_derivative.push_back(vel.row(1));
    start_end_derivative.push_back(acc.row(0));
    start_end_derivative.push_back(acc.row(1));

    Eigen::MatrixXd pos_v(2, 3);
    Eigen::MatrixXd vel_v = vel;
    Eigen::MatrixXd acc_v = acc;

    pos_v.row(0) = traj_pts.front();
    pos_v.row(1) = traj_pts.back();

    ros::Time t1, t2;
    t1 = ros::Time::now();    
    /* ========================================================================== */
    _bspline_replanner->setInput(_sdf_map, time_interval, traj_pts, start_end_derivative);
    _bspline_replanner->resetLambda2();
    bool traj_feas = false;
    bool traj_safe = false;
    Eigen:Vector3d collision_pt;
    NonUniformBspline traj_opt;

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::BsplineParameterize(time_interval, traj_pts,
                                         start_end_derivative, ctrl_pts);
    visBsplineInitialization(ctrl_pts);

    int iter = 0;
    while (iter < 5)
    {
        if (traj_feas && traj_safe) 
            break;

      /* ---------- call replanning ---------- */
        _bspline_replanner->optimize(true);
        traj_opt = _bspline_replanner->getTraj();
        BsplineFeasibleCheck(traj_opt, traj_feas, traj_safe, collision_pt);

      /* ---------- fill local minima; enlarge lambda2 ---------- */
      if (!traj_safe){
        _bspline_replanner->renewLambda2(0.05);
      }

        iter++;
    }

    //cout<<"optimization iteration: "<<iter<<endl;
    t2 = ros::Time::now();
    //cout << "replan time:" << (t2 - t1).toSec() << endl;
    /* ========================================================================== */

    /* publish optimized B-spline */
    if( traj_safe ){ 
        quadrotor_msgs::Bspline safe_spline = getBsplineTraj(traj_opt);
        double t_bspline_cmd_start, t_bspline_cmd_end;
        traj_opt.getRegion(t_bspline_cmd_start, t_bspline_cmd_end);

        double replan_traj_time = t_bspline_cmd_end - t_bspline_cmd_start;
        double time_extra = replan_traj_time - _replan_time_length;

        safe_spline.time_extra = time_extra;
        safe_spline.replan_to_global_time = local_to_global_time;

        _bspline_pub.publish(safe_spline);
        _on_replan = true;

        drawBspline(traj_opt, OPT_TRAJ_ID + succ_replan_num_, Eigen::Vector4d(0, 1, 0, 1), true,
              0.1, OPT_PT_ID + succ_replan_num_, Eigen::Vector4d(1, 1, 0, 1));
        succ_replan_num_ += 3;
    }
    else{
        // do nothing
        //cout<<"no safe replan, try it in next loop"<<endl;
    }
}

void BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt)
{   
    Vector3d pos, vel, acc;
    double t_bspline_start, t_bspline_end, t_duration;
    NonUniformBspline vel_traj = traj.getDerivative();  
    NonUniformBspline acc_traj = vel_traj.getDerivative();  

    traj.getRegion(t_bspline_start, t_bspline_end);   
    t_duration = t_bspline_end - t_bspline_start;

    is_safe = true;
    is_feas = true;
    collision_pt.setZero();
	
    Eigen::Vector3d grad;
    double dis;
    for(double t = 0.0; t < t_duration; t += 0.05)
    {
        pos =     traj.evaluateDeBoor(t_bspline_start + t);
        vel = vel_traj.evaluateDeBoor(t_bspline_start + t);
        acc = acc_traj.evaluateDeBoor(t_bspline_start + t);

        for(int i = 0; i < 3; i++){
            if( fabs(vel(i)) > _MAX_Vel || fabs(acc(i)) > _MAX_Acc )
                is_feas = false;
        }
	
        dis = _sdf_map->getDistWithGradTrilinear(pos,  grad );
	   if( dis < 0.075 ){
            is_safe = false;
            if(collision_pt.norm() < 1e-5) collision_pt = pos;
        }
    }

}

void visualizeESDFGrid(const ros::TimerEvent& event)
{   
    if( !_sdf_map->hasDepthObservation() ) 
        return;

    _sdf_map->publishMap();
    _sdf_map->publishMapInflate();
    _sdf_map->publishUpdateRange();
    _sdf_map->publishESDF();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_local_replanner");
    ros::NodeHandle nh("~");

    nh.param("local_planner/time_emergency",      _time_emergency,      1.0 );
    nh.param("local_planner/max_vel",             _MAX_Vel,             1.0 );
    nh.param("local_planner/max_acc",             _MAX_Acc,             1.0 );
    nh.param("local_planner/vis/esdf_grid_freq",  _esdf_grid_freq,      1.0 );
    nh.param("local_planner/vis/vis_traj_width",  _vis_traj_width,      0.15);
    nh.param("local_planner/vis/frame_id",        _frame_id, string("world"));

    // replan request subscriber
    _mission_sub = nh.subscribe("replan_mission", 1, rcvReplanRequestCallBack);

    // replan trajectory publisher
    _bspline_pub               = nh.advertise<quadrotor_msgs::Bspline>("bspline", 10);

    // visualization publisher
    _vis_vel_pub               = nh.advertise<visualization_msgs::Marker>("boundary_vel_vis", 1);    
    _vis_acc_pub               = nh.advertise<visualization_msgs::Marker>("boundary_acc_vis", 1);    
    _vis_fill_esdf_pub         = nh.advertise<visualization_msgs::Marker>("fill_esdf_vis",    1);    
    _vis_check_traj_pub        = nh.advertise<visualization_msgs::Marker>("check_trajectory_vis",  100);    
    _vis_initial_b_spline      = nh.advertise<visualization_msgs::Marker>("initial_b_spline_vis",  1);    
    _vis_opt_traj_pub          = nh.advertise<visualization_msgs::Marker>("replaned_b_spline_vis", 1);
    
    // inner esdf visualization timer    
    _vis_esdf_grid_timer       = nh.createTimer(ros::Duration(_esdf_grid_freq), visualizeESDFGrid);

    _bspline_replanner.reset(new BsplineReplanner());
    _bspline_replanner->initialize(_MAX_Vel, _MAX_Acc);

    _sdf_map.reset(new SDFMap);
    _sdf_map->initMap(nh);

    ros::spin();
    return 0;
}

void visCheckTrajPoints(const vector<Vector3d> & traj_pts)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = _frame_id;

    traj_vis.id = 0;
    traj_vis.ns = "local_replan/local_trajectory";
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
    traj_vis.color.r = 0.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 1.0;

    double traj_len = 0.0;
    Vector3d cur(0.0, 0.0, 0.0), pre(0.0, 0.0, 0.0);
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;
    for(int i = 0; i < (int)traj_pts.size(); i++)
    {   
        auto coord = traj_pts[i];
        cur(0) = pt.x = coord(0);
        cur(1) = pt.y = coord(1);
        cur(2) = pt.z = coord(2);
        traj_vis.points.push_back(pt);

        traj_len += (pre - cur).norm();
        pre = cur;
    }

    _vis_check_traj_pub.publish(traj_vis);
}

void displayPathWithColor(vector<Eigen::Vector3d> path,
                          Eigen::Vector4d color, int id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = _frame_id;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = id;
    mk.ns = "local_replan/traj" + to_string(id);

    //_vis_opt_traj_pub.publish(mk);
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = _vis_traj_width / 2.0;
    mk.scale.y = _vis_traj_width / 2.0;
    mk.scale.z = _vis_traj_width / 2.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
    }
    _vis_opt_traj_pub.publish(mk);
}

void drawBspline(NonUniformBspline bspline, int id,
                 Eigen::Vector4d color, bool show_ctrl_pts = false,
                 double size2 = 0.1, int id2 = 0,
                 Eigen::Vector4d color2 = Eigen::Vector4d(1, 1, 0, 1)) 
{
    vector<Eigen::Vector3d> traj_pts;
    double tm, tmp;
    bspline.getRegion(tm, tmp);
    for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
    }
    displayPathWithColor(traj_pts, color, id);

    // draw the control point
    if (!show_ctrl_pts) return;

    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
    }
    displayPathWithColor(ctp, color2, id2);
}

void visBsplineInitialization(const MatrixXd & ctrl_pts)
{
    visualization_msgs::Marker ctrl_pts_vis;

    ctrl_pts_vis.header.stamp       = ros::Time::now();
    ctrl_pts_vis.header.frame_id    = _frame_id;

    ctrl_pts_vis.id = 0;
    ctrl_pts_vis.ns = "local_replan/B-spline_initialization";
    ctrl_pts_vis.type = visualization_msgs::Marker::SPHERE_LIST;

    ctrl_pts_vis.action = visualization_msgs::Marker::ADD;
    ctrl_pts_vis.scale.x = _vis_traj_width / 2.0;
    ctrl_pts_vis.scale.y = _vis_traj_width / 2.0;
    ctrl_pts_vis.scale.z = _vis_traj_width / 2.0;
    ctrl_pts_vis.pose.orientation.x = 0.0;
    ctrl_pts_vis.pose.orientation.y = 0.0;
    ctrl_pts_vis.pose.orientation.z = 0.0;
    ctrl_pts_vis.pose.orientation.w = 1.0;

    ctrl_pts_vis.color.a = 1.0;
    ctrl_pts_vis.color.r = 1.0;
    ctrl_pts_vis.color.g = 0.0;
    ctrl_pts_vis.color.b = 0.0;
    
    ctrl_pts_vis.points.clear();

    geometry_msgs::Point pt;
    for(int i = 0; i < (int)ctrl_pts.rows(); i++)
    {   
        pt.x = ctrl_pts(i, 0);
        pt.y = ctrl_pts(i, 1);
        pt.z = ctrl_pts(i, 2);
        ctrl_pts_vis.points.push_back(pt);
    }

    //ROS_INFO("[local_replan] The length of the trajectory; %fm.", traj_len);
    _vis_initial_b_spline.publish(ctrl_pts_vis);
}

void visFillMinima(Eigen::Vector3d center, Eigen::Vector3d sc) {
  static int idx = 0;
  idx++;

  visualization_msgs::Marker cube;
  cube.header.frame_id = _frame_id;
  cube.header.stamp = ros::Time::now();
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.id = idx;

  cube.pose.orientation.w = 1.0;
  cube.color.b = 1.0, cube.color.a = 1.0, cube.color.r = 0.0,
  cube.color.g = 0.0;

  cube.pose.position.x = center(0);
  cube.pose.position.y = center(1);
  cube.pose.position.z = center(2);

  cube.scale.x = sc(0);
  cube.scale.y = sc(1);
  cube.scale.z = sc(2);

  _vis_fill_esdf_pub.publish(cube);
}
