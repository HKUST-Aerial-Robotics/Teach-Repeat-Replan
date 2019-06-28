#include "ros/ros.h"
#include "ros/console.h"
#include <deque>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <grad_replanner/bezier_base.h>
#include <grad_replanner/grad_band_optimizer.h>
#include <grad_replanner/non_uniform_bspline.h>
#include <quadrotor_msgs/Bspline.h>
#include <quadrotor_msgs/ReplanCheck.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SpatialTemporalTrajectory.h>
#include <grad_replanner/backward.hpp>

const int  _DIM_x = 0;
const int  _DIM_y = 1;
const int  _DIM_z = 2;

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

class Bspline_stamped{

public:
    ros::Time plan_start_time;
    NonUniformBspline spline[3];
    int traj_id;
    ros::Time local_traj_end_time;
    double t_bspline_cmd_start;

    Bspline_stamped(ros::Time plan_start_time_, ros::Time local_traj_end_time_, double t_bspline_cmd_start_, NonUniformBspline spline_[3], int traj_id_): plan_start_time(plan_start_time_), traj_id(traj_id_), local_traj_end_time(local_traj_end_time_), t_bspline_cmd_start(t_bspline_cmd_start_){
        for(int i=0; i<3; i++)
            spline[i] = spline_[i];
    }

};

class TrajectoryServer
{
private:

    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _time_sub;
    ros::Subscriber _bspline_sub;
    ros::Subscriber _transformation_sub;
    ros::Subscriber _trigger_sub;

    // publishers
    ros::Publisher _start_pose_pub;
    ros::Publisher _cmd_pub;
    ros::Publisher _yaw_pub;
    ros::Publisher _vis_yaw_pub;
    ros::Publisher _vis_pos_pub;
    ros::Publisher _vis_vel_pub;
    ros::Publisher _vis_acc_pub;
    ros::Publisher _current_pose_pub;
    ros::Publisher _check_pts_pub;
    
    // configuration for trajectory
    uint32_t traj_flag = 0;
    int traj_id   = 0;
    int time_id   = 0;
    int n_segment = 0;
    int poly_num1d;

    //bool replan_cmd = false;
    // the global spatial trajectory: Bezier Curve
    VectorXd range;
    MatrixXd coef;

    // the global temporal trajectory: Time Allocator
    double s_step; 
    int K_max;
    MatrixXd a_data, b_data;        
    MatrixXd time, time_acc, s_data; 
    VectorXd K_data;

    // the replan B-spline trajectory: B-Spline
    std::deque<Bspline_stamped> replan_trajs, replan_trajs_view;
    int replan_id = -1;

    ros::Time global_traj_start_time = ros::TIME_MAX;
    ros::Time global_traj_final_time = ros::TIME_MIN;
    //ros::Time replan_traj_start_time = ros::TIME_MAX;

    ros::Time odom_time = ros::TIME_MAX;

    double global_traj_time;
    double replan_traj_time;

    double local_to_global_time;
    double global_to_local_time;
    double local_end_time;

    double local_to_global_time_view;
    double local_end_time_view;

    int traj_order;
    double check_collision_time_interval;// = 0.1;
    double time_commit;//   = 0.0;
    double time_horizon;//  = 1.0;
    double time_lead_yaw;// = 0.0;
    bool pub_lead_yaw       = false;
    bool use_lead_yaw_plan1 = false;

    ros::Timer check_collision_timer;

    Matrix3d w_R_odom, w_R_odom_replan;
    Vector3d w_t_odom, w_t_odom_replan;

    bool update_transform;
    enum ServerState{INIT, TRAJ, HOVER} state = INIT;
    nav_msgs::Odometry odom;
    quadrotor_msgs::PositionCommand cmd;
    geometry_msgs::Point yaw;

    visualization_msgs::Marker vis_pos, vis_vel, vis_acc, vis_yaw;

    double vis_yaw_length = 4.0;
    double last_yaw = 0.0;
public:
    
    VectorXd C, Cv, Ca; 
    Bernstein * bezier_basis = NULL;

    TrajectoryServer(ros::NodeHandle & handle, 
        int traj_order_, double check_collision_time_interval_, double time_commit_, double time_horizon_, double time_lead_yaw_, bool pub_lead_yaw_, bool use_lead_yaw_plan1_): 
        traj_order(traj_order_), check_collision_time_interval(check_collision_time_interval_), 
        time_commit(time_commit_), time_horizon(time_horizon_), time_lead_yaw(time_lead_yaw_), pub_lead_yaw(pub_lead_yaw_), use_lead_yaw_plan1(use_lead_yaw_plan1_) 
    {   
        _odom_sub = 
            handle.subscribe("odometry",   1, &TrajectoryServer::rcvOdometryCallBack,    this);

        _traj_sub =
            handle.subscribe("trajectory", 1, &TrajectoryServer::rcvTrajectoryCallBack,  this, ros::TransportHints().tcpNoDelay());

        _bspline_sub =
            handle.subscribe("bspline",    1, &TrajectoryServer::rcvBsplineCallBack,     this);

        _transformation_sub =
            handle.subscribe("w_T_odom",   1, &TrajectoryServer::rcvTransformCallBack,   this);

        _trigger_sub =
            handle.subscribe("trigger",   10, &TrajectoryServer::rcvTriggerCallBack,     this);

        
        _start_pose_pub =
            handle.advertise<geometry_msgs::PoseStamped>("start_pose", 50);

        _cmd_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        _current_pose_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("current_pose", 50);

        _yaw_pub = 
            handle.advertise<geometry_msgs::Point>("view", 50);

        _vis_pos_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_position", 50);
        
        _vis_vel_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_velocity", 50);
        
        _vis_acc_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_yaw_pub = 
            handle.advertise<visualization_msgs::Marker>("vis_yaw_cmd", 50);

        _check_pts_pub = 
            handle.advertise<quadrotor_msgs::ReplanCheck>("check_pts", 50);

        check_collision_timer = 
            handle.createTimer(ros::Duration(check_collision_time_interval), &TrajectoryServer::checkCollisionCallBack, this);
        
        bezier_basis = new Bernstein(3.0);

        double pos_gain[3] = {5.7, 5.7, 6.2};
        double vel_gain[3] = {3.4, 3.4, 4.0};
        setGains(pos_gain, vel_gain);

        w_R_odom = Matrix3d::Identity();
        w_R_odom_replan = Matrix3d::Identity();
        w_t_odom = Vector3d::Zero();
        w_t_odom_replan = Vector3d::Zero();

        update_transform = true;

        poly_num1d = traj_order + 1;

        vis_pos.ns = "pos";
        vis_pos.id = 0;
        vis_pos.header.frame_id = "/world";
        vis_pos.type = visualization_msgs::Marker::SPHERE;
        vis_pos.action = visualization_msgs::Marker::ADD;
        vis_pos.color.a = 1.0;
        vis_pos.color.r = 0.0;
        vis_pos.color.g = 0.0;
        vis_pos.color.b = 0.0;
        vis_pos.scale.x = 0.3;
        vis_pos.scale.y = 0.3;
        vis_pos.scale.z = 0.3;

        vis_vel.ns = "vel";
        vis_vel.id = 0;
        vis_vel.header.frame_id = "/world";
        vis_vel.type = visualization_msgs::Marker::ARROW;
        vis_vel.action = visualization_msgs::Marker::ADD;
        vis_vel.color.a = 1.0;
        vis_vel.color.r = 0.0;
        vis_vel.color.g = 1.0;
        vis_vel.color.b = 0.0;
        vis_vel.scale.x = 0.1;
        vis_vel.scale.y = 0.2;
        vis_vel.scale.z = 0.2;

        vis_acc.ns = "acc";
        vis_acc.id = 0;
        vis_acc.header.frame_id = "/world";
        vis_acc.type = visualization_msgs::Marker::ARROW;
        vis_acc.action = visualization_msgs::Marker::ADD;
        vis_acc.color.a = 1.0;
        vis_acc.color.r = 1.0;
        vis_acc.color.g = 1.0;
        vis_acc.color.b = 0.0;
        vis_acc.scale.x = 0.1;
        vis_acc.scale.y = 0.2;
        vis_acc.scale.z = 0.2;

        vis_yaw.ns = "yaw";
        vis_yaw.id = 0;
        vis_yaw.header.frame_id = "/world";
        vis_yaw.type = visualization_msgs::Marker::ARROW;
        vis_yaw.action = visualization_msgs::Marker::ADD;
        vis_yaw.color.a = 1.0;
        vis_yaw.color.r = 0.0;
        vis_yaw.color.g = 0.0;
        vis_yaw.color.b = 1.0;
        vis_yaw.scale.x = 0.12;
        vis_yaw.scale.y = 0.24;
        vis_yaw.scale.z = 0.24;

        cout<<"check_collision_time_interval:= "<<check_collision_time_interval<<endl;
        cout<<"time_commit := " <<time_commit<<endl;
        cout<<"time_horizon:= " <<time_horizon<<endl;
        cout<<"time_lead_yaw:= "<<time_lead_yaw<<endl;
        cout<<"pub_lead_yaw:= " <<pub_lead_yaw<<endl;
    }

    void setGains(double pos_gain[3], double vel_gain[3])
    {
        cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    void rcvOdometryCallBack(const nav_msgs::Odometry & msg)
    {   
        // #1. store the odometry
        odom = msg;
        odom_time = odom.header.stamp;

        if(state == INIT )
        {
            cmd.position   = odom.pose.pose.position;   
          
            cmd.header.stamp = odom.header.stamp;
            cmd.header.frame_id = "/map";
            cmd.trajectory_flag = traj_flag;

            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
            _cmd_pub.publish(cmd);

            return;
        }

        if( replan_trajs.size() > 0 && odom_time.toSec() >= replan_trajs.back().local_traj_end_time.toSec()){
            global_traj_start_time = global_traj_start_time + ros::Duration(local_end_time - local_to_global_time);
        } 

        // #2. try to publish command
        pubPositionCommand();

        // #3. try to calculate the new state
        if (state == TRAJ && global_traj_start_time!= ros::TIME_MAX && ( (odom_time - global_traj_start_time).toSec() > global_traj_time ) && replan_trajs.size() > 0 && (odom_time - replan_trajs.back().local_traj_end_time).toSec() > 0.0 )
        {
            state = HOVER;
            traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }

    void rcvTrajectoryCallBack(const quadrotor_msgs::SpatialTemporalTrajectory & traj)
    {
        //ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", global_traj_start_time.toSec());
        // #1. try to execuse the action
        if (traj.action == quadrotor_msgs::SpatialTemporalTrajectory::ACTION_ADD)
        {   
            //ROS_WARN("[SERVER] Recevied The Bezier Curve with, segments num : %d.", traj.num_segment);
            //if ((int)traj.trajectory_id < traj_id) return ;

            state = TRAJ;
            traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            traj_id   = traj.trajectory_id;

            n_segment = traj.num_segment;
            
            coef = MatrixXd::Zero(n_segment, 3*(traj_order + 1) );
            range.resize(n_segment);
            
            //ROS_WARN("stack the coefficients");
            int poly_num = traj_order + 1;
            for (int idx = 0; idx < n_segment; ++idx)
            {   
                for (int j = 0; j < poly_num; ++j)
                {
                    coef(idx, 0 * poly_num + j) = traj.coef_x[idx * poly_num + j];
                    coef(idx, 1 * poly_num + j) = traj.coef_y[idx * poly_num + j];
                    coef(idx, 2 * poly_num + j) = traj.coef_z[idx * poly_num + j];
                }
                range(idx) = traj.range[idx];
            }

            global_traj_start_time = traj.start_time;
            global_traj_final_time = traj.final_time;
            global_traj_time       = (global_traj_final_time - global_traj_start_time).toSec();

            int seg_num = traj.K.size();
            K_max  = traj.K_max;
            s_step = traj.s_step;

            K_data = VectorXd::Zero(seg_num);
            a_data = MatrixXd::Zero(seg_num, K_max);
            b_data = MatrixXd::Zero(seg_num, K_max + 1);
            s_data = MatrixXd::Zero(seg_num, K_max + 1);
            
            time     = MatrixXd::Zero(seg_num, K_max);
            time_acc = MatrixXd::Zero(seg_num, K_max);

            int K_shift = 0;
            int K_plus_shift = 0;
            for (int i = 0; i < seg_num; i++ )
            {     
                K_data(i) = traj.K[i];

                for(int j = 0; j < K_data(i) + 1; j++ )
                {   
                    if( j < K_data(i) )
                    {
                        a_data(i, j) = traj.a[j + K_shift];
                        time    (i, j) = traj.time[j + K_shift];
                        time_acc(i, j) = traj.time_acc[j + K_shift];
                    }

                    b_data(i, j) = traj.b[j + K_plus_shift];
                    s_data(i, j) = traj.s[j + K_plus_shift];
                }

                K_shift      += K_data(i);
                K_plus_shift += K_data(i) + 1;
            }
        }
        else if (traj.action == quadrotor_msgs::SpatialTemporalTrajectory::ACTION_ABORT) 
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::SpatialTemporalTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }

        update_transform = true;
        ROS_WARN("finish loading global curve");
    }


    void rcvBsplineCallBack(quadrotor_msgs::BsplineConstPtr msg) 
    {   
        if(replan_id >= msg->traj_id) return;

        if (msg->action == quadrotor_msgs::Bspline::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
            ROS_WARN("[Server] Received stop trajectory, stopping");
        }
        else
        {
            ROS_WARN("[Server] Recevied a safe B-Spline");
            w_R_odom_replan = w_R_odom;
            w_t_odom_replan = w_t_odom;

            Eigen::VectorXd knots(msg->knots.size());
            for (int i = 0; i < (int)msg->knots.size(); ++i) {
                knots(i) = msg->knots[i];
            }

            Eigen::MatrixXd ctrl_pts(msg->pts.size(), 3);
            for (int i = 0; i < (int)msg->pts.size(); ++i) {
                Eigen::Vector3d pt;
                pt(0) = msg->pts[i].x;
                pt(1) = msg->pts[i].y;
                pt(2) = msg->pts[i].z;
                ctrl_pts.row(i) = pt.transpose();
            }

            NonUniformBspline bspline(ctrl_pts, msg->order, 0.1);
            bspline.setKnot(knots);

            //replan_traj_start_time = msg->start_time;
            replan_id  = msg->traj_id;

            NonUniformBspline spline_full[3];
            spline_full[0] = bspline;
            spline_full[1] = spline_full[0].getDerivative();
            spline_full[2] = spline_full[1].getDerivative();

            double cmd_start_time = 0.0, cmd_end_time = 0.0;
            bspline.getRegion(cmd_start_time, cmd_end_time);
            //cout<<"time extra?:= "<<cmd_end_time - cmd_start_time - time_horizon<<endl;
            
            if(replan_trajs.size() > 0){
                replan_trajs.back().local_traj_end_time = msg->start_time;
            }

            replan_trajs.push_back(Bspline_stamped(msg->start_time, msg->start_time + ros::Duration(cmd_end_time - cmd_start_time), cmd_start_time, spline_full, msg->traj_id));

            local_end_time = replan_trajs.back().local_traj_end_time.toSec();
            local_to_global_time = msg -> replan_to_global_time;
            if( odom_time.toSec() < replan_trajs.front().plan_start_time.toSec()){
                    global_to_local_time = replan_trajs.front().plan_start_time.toSec();
            }

            /*  for the leading yaw command  */
            /*if(replan_trajs_view.size() > 0){
                replan_trajs_view.back().local_traj_end_time = msg->start_time;// + ros::Duration(time_lead_yaw);
            }
            replan_trajs_view.push_back(Bspline_stamped(msg->start_time + ros::Duration(time_lead_yaw), 
                msg->start_time + ros::Duration(cmd_end_time - cmd_start_time), cmd_start_time, spline_full, msg->traj_id));
            local_end_time_view = replan_trajs_view.back().local_traj_end_time.toSec() + time_lead_yaw;
            local_to_global_time_view = msg -> replan_to_global_time;// + time_lead_yaw;*/

            replan_trajs_view = replan_trajs;
            /*  ***********************************  */
        }
        // b_splines.push_back(bspline);
        // b_splines.push_back(b_splines[0].getDerivative());
        // b_splines.push_back(b_splines[1].getDerivative());
        /*cout<<"t_bspline_cmd_start: "<<t_bspline_cmd_start<<endl;
        cout<<"t_bspline_cmd_end: "  <<t_bspline_cmd_end  <<endl;*/

        //replan_traj_time = t_bspline_cmd_end - t_bspline_cmd_start;
        //replan_cmd = true;
        //receive_traj = true;
    }

    void rcvTransformCallBack(geometry_msgs::PoseConstPtr msg){
        if(!update_transform) return;
        Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        w_R_odom = q.toRotationMatrix();
        
        w_t_odom << msg->position.x, msg->position.y, msg->position.z;
    }

    void rcvTriggerCallBack(geometry_msgs::PoseStampedConstPtr msg){
        update_transform = false;
        geometry_msgs::PoseStamped start_pose;
        start_pose.header = msg->header;
        Vector3d start_position(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        start_position = w_R_odom * start_position + w_t_odom;
        start_pose.pose.position.x = start_position.x();
        start_pose.pose.position.y = start_position.y();
        start_pose.pose.position.z = start_position.z();

        Quaterniond start_orientation(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
        start_orientation = Quaterniond(w_R_odom) * start_orientation;
        start_pose.pose.orientation.w = start_orientation.w();
        start_pose.pose.orientation.x = start_orientation.x();
        start_pose.pose.orientation.y = start_orientation.y();
        start_pose.pose.orientation.z = start_orientation.z();

        _start_pose_pub.publish(start_pose);
    }

    void checkCollisionCallBack(const ros::TimerEvent&){

        if( odom_time == ros::TIME_MAX || traj_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) return;

        quadrotor_msgs::ReplanCheck check_traj;
        check_traj.header.stamp = odom_time + ros::Duration(time_commit);
        check_traj.trajectory_id = replan_id + 1;
        check_traj.check_points_time_interval = 0.02;

        unsigned int local_sec = 0;
        double mean_speed = 0.0;
        double final_time = 0.0;

        double temp_local_to_global_time = -1.0;

        Eigen::Vector3d position, velocity, acceleration;
        geometry_msgs::Point point_temp;

        bool flag = 0;
        for(double t = odom_time.toSec() + time_commit; t <= odom_time.toSec() + time_commit + time_horizon || (replan_trajs.size() > 0 && t <= replan_trajs.back().local_traj_end_time.toSec()); t+=0.02){
            double t_ = 0.0;
            
            if( replan_trajs.size() > 0){
                t_= t - replan_trajs[local_sec].plan_start_time.toSec();

                double t_start = t - replan_trajs.front().plan_start_time.toSec();
                double t_end = t - replan_trajs.back().local_traj_end_time.toSec();

                if( t_start < 0.0){
                    t_ = t - global_traj_start_time.toSec();
                    if(getGlobalCommand(t_, position, velocity, acceleration)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.check_points.push_back(point_temp);
                        final_time = t;
                        mean_speed += velocity.norm();

                    }
                }
                else if( t_end > 0.0){
                    t_ = t - local_end_time + local_to_global_time - global_traj_start_time.toSec();

                    if(flag == 0){
                        // cout << "to global_t :" << t_ <<endl;
                        flag = 1;
                    }

                    if(getGlobalCommand(t_, position, velocity, acceleration)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.check_points.push_back(point_temp);
                        final_time = t;
                        mean_speed += velocity.norm();

                        temp_local_to_global_time = t_ + global_traj_start_time.toSec();
                    }
                }
                else if (t_start >= 0.0 && t_end <= 0.0){
                    if( t > replan_trajs[local_sec].local_traj_end_time.toSec()) {
                        local_sec ++;
                        t -= 0.02;
                        continue;
                    }

                    if(getLocalCommand(t_, position, velocity, acceleration, local_sec)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.check_points.push_back(point_temp);
                        final_time = t;
                        mean_speed += velocity.norm();
                    }
                }
                
            }
            else{
                t_ = t - global_traj_start_time.toSec();

                // cout << "global_t :" << t_ <<endl;

                if(getGlobalCommand(t_, position, velocity, acceleration)){
                    position = w_R_odom.transpose() * (position - w_t_odom);
                    
                    point_temp.x = position.x();
                    point_temp.y = position.y();
                    point_temp.z = position.z();

                    check_traj.check_points.push_back(point_temp);
                    final_time = t;
                    mean_speed += velocity.norm();
                }
                local_to_global_time = t;
            }
        }

        if(check_traj.check_points.size() > 0)
        {
            mean_speed /= check_traj.check_points.size();
            check_traj.replan_time_length = final_time - odom_time.toSec() - time_commit;
            //check_traj.plan_points_time_interval = 0.3 / mean_speed;
            check_traj.plan_points_time_interval = min(max( 0.2 , 0.3 / mean_speed ), 0.4 );
        }
        else
            return;

        //ROS_WARN("mean_speed: = %f, plan_points_time_interval: = %f", mean_speed, check_traj.plan_points_time_interval);

        int plan_pts = ceil( check_traj.replan_time_length / check_traj.plan_points_time_interval);
        check_traj.plan_points_time_interval = check_traj.replan_time_length / plan_pts;

        local_sec = 0;
        int i = 0;
        for(double t = odom_time.toSec() + time_commit; i <= plan_pts; t+=check_traj.plan_points_time_interval, i++){
            
            double t_ = 0.0;
            
            if( replan_trajs.size() > 0){
                t_ = t - replan_trajs[local_sec].plan_start_time.toSec();

                double t_start = t - replan_trajs.front().plan_start_time.toSec();
                double t_end = t - replan_trajs.back().local_traj_end_time.toSec();

                if( t_start < 0.0){
                    t_ = t - global_traj_start_time.toSec();
                    if(getGlobalCommand(t_, position, velocity, acceleration)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.plan_points.push_back(point_temp);

                    }
                }
                else if( t_end > 0.0){
                    t_ = t - local_end_time + local_to_global_time - global_traj_start_time.toSec();

                    if(getGlobalCommand(t_, position, velocity, acceleration)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.plan_points.push_back(point_temp);
                    }
                }
                else if (t_start >= 0.0 && t_end <= 0.0){
                    if( t > replan_trajs[local_sec].local_traj_end_time.toSec()) {
                        local_sec ++;
                        t -= check_traj.plan_points_time_interval;
                        i --;
                        continue;
                    }

                    if(getLocalCommand(t_, position, velocity, acceleration, local_sec)){
                        position = w_R_odom.transpose() * (position - w_t_odom);
                        
                        point_temp.x = position.x();
                        point_temp.y = position.y();
                        point_temp.z = position.z();

                        check_traj.plan_points.push_back(point_temp);
                    }
                }

            }
            else{
                t_ = t - global_traj_start_time.toSec();
                if(getGlobalCommand(t_, position, velocity, acceleration)){
                    position = w_R_odom.transpose() * (position - w_t_odom);
                    
                    point_temp.x = position.x();
                    point_temp.y = position.y();
                    point_temp.z = position.z();

                    check_traj.plan_points.push_back(point_temp);
                }
            }

            if(i == 0){
                velocity = w_R_odom.transpose() * velocity;
                check_traj.start_velocity.x = velocity.x(); 
                check_traj.start_velocity.y = velocity.y(); 
                check_traj.start_velocity.z = velocity.z(); 

                acceleration = w_R_odom.transpose() * acceleration;
                check_traj.start_acceleration.x = acceleration.x();
                check_traj.start_acceleration.y = acceleration.y();
                check_traj.start_acceleration.z = acceleration.z();
            }
            else if(i == plan_pts){
                velocity = w_R_odom.transpose() * velocity;
                check_traj.stop_velocity.x = velocity.x(); 
                check_traj.stop_velocity.y = velocity.y(); 
                check_traj.stop_velocity.z = velocity.z(); 

                acceleration = w_R_odom.transpose() * acceleration;
                check_traj.stop_acceleration.x = acceleration.x();
                check_traj.stop_acceleration.y = acceleration.y();
                check_traj.stop_acceleration.z = acceleration.z();
            }
        }

        check_traj.replan_to_global_time = max(temp_local_to_global_time, local_to_global_time);
        _check_pts_pub.publish(check_traj);
    }

    bool getGlobalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration)
    {   
        if( t < 0.0 || t > global_traj_time) 
        {   
            //cout<<"t:= "<<t<<endl;
            return false;
        }

        int idx;
        for(idx = 0; idx < n_segment; idx++)
        {   
            int K = K_data(idx);
            if( t  > time(idx, K - 1)) t -= time(idx, K - 1);
            else break;
        }

        double t_tmp = t;     
        int grid_num = K_data(idx);

        //ROS_WARN("[Time Optimal Server] publish command, segm index is %d, segm time is %f", idx, t);
        int grid_idx;
        for(grid_idx = 0; grid_idx < K_data(idx); grid_idx++)
        {
            if (t > time(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        
        double delta_s = t * s_step / delta_t;

        double s = s_data(idx, grid_idx) + delta_s;

        Vector3d position_s = bezier_basis->getPos(coef, idx, s/range(idx)) * range(idx); 
        position   = position_s;
        //cout<<"s: "<<s<<", position: \n"<<position<<endl;

        double s_k   = s_data(idx, grid_idx);
        double s_k_1 = s_data(idx, grid_idx + 1);
        double b_k   = b_data(idx, grid_idx);
        double b_k_1 = b_data(idx, grid_idx + 1);

        Vector3d velocity_s1 = bezier_basis->getVel(coef, idx, s_k   /range(idx)); 
        Vector3d velocity_s2 = bezier_basis->getVel(coef, idx, s_k_1 /range(idx));

        Vector3d velocity1 = velocity_s1 * sqrt(b_k);
        Vector3d velocity2 = velocity_s2 * sqrt(b_k_1);
        velocity  = velocity1 + (velocity2 - velocity1) * t / delta_t;

        t = t_tmp;
        for(grid_idx = 0; grid_idx < K_data(idx); grid_idx++)
        {
            if (t > time_acc(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time_acc(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }
        
        if(grid_idx == grid_num) t -= time_acc(idx, grid_num - 1);

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;

        double a_k, s_1;
        if( grid_idx == 0 && idx == 0 )
        {   
            s_k   = s_data(idx, 0);
            s_k_1 = s_data(idx, 0 + 1);
            
            a_k   = a_data(idx, 0);
            b_k   = b_data(idx, 0);
            b_k_1 = b_data(idx, 0 + 1);

            s_1 = (s_k + s_k_1 ) / 2.0 / range(idx);
            velocity_s     = bezier_basis->getVel(coef, idx, s_1);
            acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration1 << 0.0, 0.0, 0.0;
            
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
        }
        else if( grid_idx == grid_num && idx == (n_segment - 1) )
        {   
            s_k   = s_data(idx, grid_num - 1);
            s_k_1 = s_data(idx, grid_num);
            
            a_k   = a_data(idx, grid_num - 1);
            b_k   = b_data(idx, grid_num - 1);
            b_k_1 = b_data(idx, grid_num    );

            s_1 = (s_k + s_k_1 ) / 2.0 /range(idx);
            velocity_s     = bezier_basis->getVel(coef, idx, s_1);
            acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
            acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
        }
        else
        {   
            if(grid_idx < grid_num && grid_idx > 0) // take average accleration in a same segment
            {   
                delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
                
                s_k   = s_data(idx, grid_idx - 1);
                s_k_1 = s_data(idx, grid_idx + 0);
                
                a_k   = a_data(idx, grid_idx - 1);
                b_k   = b_data(idx, grid_idx - 1);
                b_k_1 = b_data(idx, grid_idx + 0);

                s_1 = (s_k + s_k_1 ) / 2.0 /range(idx);
                velocity_s     = bezier_basis->getVel(coef, idx, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = s_data(idx, grid_idx + 0);
                s_k_1 = s_data(idx, grid_idx + 1);

                a_k   = a_data(idx, grid_idx + 0);
                b_k   = b_data(idx, grid_idx + 0);
                b_k_1 = b_data(idx, grid_idx + 1);              

                s_1 = (s_k + s_k_1 ) / 2.0 /range(idx);
                velocity_s     = bezier_basis->getVel(coef, idx, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
            }
            else if(grid_idx == grid_num)// take average accleration between two segments
            {   
                delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
                
                s_k   = s_data(idx, grid_idx - 1);
                s_k_1 = s_data(idx, grid_idx);
                
                a_k   = a_data(idx, grid_idx - 1);
                b_k   = b_data(idx, grid_idx - 1);
                b_k_1 = b_data(idx, grid_idx);

                s_1 = (s_k + s_k_1 ) / 2.0 /range(idx);
                velocity_s     = bezier_basis->getVel(coef, idx, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                s_k   = s_data(idx + 1, 0);
                s_k_1 = s_data(idx + 1, 1);

                a_k   = a_data(idx + 1, 0);
                b_k   = b_data(idx + 1, 0);
                b_k_1 = b_data(idx + 1, 1);              

                s_1 = (s_k + s_k_1 ) / 2.0 /range(idx + 1);
                velocity_s     = bezier_basis->getVel(coef, idx + 1, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx + 1, s_1) / range(idx + 1);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
            }
            else if(grid_idx == 0)// take average accleration between two segments
            {   
                int grid_num_k = K_data(idx - 1); // last segment's grid num
                delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
                
                s_k   = s_data(idx - 1, grid_num_k - 1);
                s_k_1 = s_data(idx - 1, grid_num_k    );
                
                a_k   = a_data(idx - 1, grid_num_k - 1);
                b_k   = b_data(idx - 1, grid_num_k - 1);
                b_k_1 = b_data(idx - 1, grid_num_k    );

                s_1 = (s_k + s_k_1 ) / 2.0 / range(idx - 1);
                velocity_s     = bezier_basis->getVel(coef, idx - 1, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx - 1, s_1) / range(idx - 1);
                acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = s_data(idx, 0);
                s_k_1 = s_data(idx, 0 + 1);
                
                a_k   = a_data(idx, 0);
                b_k   = b_data(idx, 0);
                b_k_1 = b_data(idx, 0 + 1);

                s_1 = (s_k + s_k_1 ) / 2.0 / range(idx);
                velocity_s     = bezier_basis->getVel(coef, idx, s_1);
                acceleration_s = bezier_basis->getAcc(coef, idx, s_1) / range(idx);
                acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
            } 
        }
        return true;
    }

    bool getLocalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration, int replan_traj_idx)
    {
        if( t < 0.0 || t > (replan_trajs[replan_traj_idx].local_traj_end_time - replan_trajs[replan_traj_idx].plan_start_time).toSec()) return false;

        position     = w_R_odom_replan * (replan_trajs[replan_traj_idx].spline[0].evaluateDeBoor(replan_trajs.front().t_bspline_cmd_start + t)) + w_t_odom_replan;
        velocity     = w_R_odom_replan * (replan_trajs[replan_traj_idx].spline[1].evaluateDeBoor(replan_trajs.front().t_bspline_cmd_start + t));
        acceleration = w_R_odom_replan * (replan_trajs[replan_traj_idx].spline[2].evaluateDeBoor(replan_trajs.front().t_bspline_cmd_start + t));

        return true;
    }

    bool getCtrlCommand(Vector3d & position, Vector3d & velocity, Vector3d & acceleration)
    {
        double t = odom.header.stamp.toSec();
        double t_= odom.header.stamp.toSec();

        for(; replan_trajs.size() > 0 && t > (replan_trajs.front().local_traj_end_time).toSec();  replan_trajs.pop_front());

        if( replan_trajs.size() > 0)
        {
            t_= t - replan_trajs.front().plan_start_time.toSec();

            double t_start = t - replan_trajs.front().plan_start_time.toSec();
            double t_end = t - replan_trajs.back().local_traj_end_time.toSec();

            if( t_start < 0.0){
                t_ = t - global_traj_start_time.toSec();
                if(!getGlobalCommand(t_, position, velocity, acceleration)) return 0;
            }
            else if( t_end > 0.0){
                t_ = t - local_end_time + local_to_global_time - global_traj_start_time.toSec();

                if(!getGlobalCommand(t_, position, velocity, acceleration)) return 0;

            }
            else if (t_start >= 0.0 && t_end <= 0.0){
                if(!getLocalCommand(t_, position, velocity, acceleration, 0.0)) return 0;
            }
            
        }
        else
        {
            t_ = t - global_traj_start_time.toSec();
            if(!getGlobalCommand(t_, position, velocity, acceleration)) return 0;
        }

        return 1;
    }

    bool getViewCtrlCommand(Vector3d & position, Vector3d & velocity, Vector3d & acceleration, double time_bias)
    {
        double t  = odom_time.toSec() + time_bias;
        double t_ = 0.0;
        
        int local_sec = 0;
        if( replan_trajs.size() > 0){
            //t_ = t - replan_trajs[local_sec].plan_start_time.toSec();
            double t_start = t - replan_trajs.front().plan_start_time.toSec();
            double t_end   = t - replan_trajs.back().local_traj_end_time.toSec();

            if (t_start >= 0.0 && t_end <= 0.0){
                while( t > replan_trajs[local_sec].local_traj_end_time.toSec()) {
                    local_sec ++;
                }
                
                t_ = t - replan_trajs[local_sec].plan_start_time.toSec();
                if(!getLocalCommand(t_, position, velocity, acceleration, local_sec))
                    return 0;
            }

            else if( t_start < 0.0){
                t_ = t - global_traj_start_time.toSec();
                if(!getGlobalCommand(t_, position, velocity, acceleration)) 
                    return 0;
            }

            else if( t_end > 0.0){
                t_ = t - local_end_time + local_to_global_time - global_traj_start_time.toSec();
                if( !getGlobalCommand(t_, position, velocity, acceleration))
                    return 0;
            }

        }
        else{
            t_ = t - global_traj_start_time.toSec();
            if( !getGlobalCommand(t_, position, velocity, acceleration))
                return 0;
        }

        position = w_R_odom.transpose() * (position - w_t_odom);
        velocity = w_R_odom.transpose() * velocity;
        acceleration = w_R_odom.transpose() * acceleration;

        return 1;
    }

    void pubPositionCommand()
    {
        // #1. check if it is right state
        if (state == INIT) return;
        else if (state == HOVER )
        {
            cmd.header.stamp = odom.header.stamp;
            cmd.header.frame_id = "/map";
            cmd.trajectory_flag = traj_flag;

            cmd.position = odom.pose.pose.position;
            cmd.yaw = atan2(2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y), 1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
            cmd.yaw_dot = 1.0;

            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
        }
        // #2. locate the trajectory segment
        else if ( state == TRAJ )
        {   
            cmd.header.stamp = odom.header.stamp;
            cmd.header.frame_id = "/map";
            cmd.trajectory_flag = traj_flag;
            cmd.trajectory_id   = traj_id;
            /*cmd.yaw_dot = 0.0;
            cmd.yaw = 0.0;*/
            Vector3d position,      velocity,      acceleration;
            Vector3d position_view, velocity_view, acceleration_view;

            bool use_last_yaw = false;
            // #3. calculate the desired states
            if( !getCtrlCommand(position, velocity, acceleration))
            {   
                _yaw_pub.publish(yaw);
                _cmd_pub.publish(cmd);
                return;
            }
            
            if(pub_lead_yaw){
                if( !getViewCtrlCommand(position_view, velocity_view, acceleration_view, time_lead_yaw)){
                    use_last_yaw = true;
                }
            }

            position = w_R_odom.transpose() * (position - w_t_odom);
            velocity = w_R_odom.transpose() * velocity;
            acceleration = w_R_odom.transpose() * acceleration;

            cmd.position.x = position(0);
            cmd.position.y = position(1);
            cmd.position.z = position(2);

            cmd.velocity.x = velocity(0);
            cmd.velocity.y = velocity(1);
            cmd.velocity.z = velocity(2);
            
            cmd.acceleration.x = acceleration(0);
            cmd.acceleration.y = acceleration(1);
            cmd.acceleration.z = acceleration(2);

            cmd.yaw = atan2(velocity(1), velocity(0));
            cmd.yaw_dot = 1.0; //(fabs(velocity(0))<min_follow_yaw_speed && fabs(velocity(1))<min_follow_yaw_speed) ? -1.0 : 1.0;    

            yaw.x = velocity(0) / velocity.head(2).norm();
            yaw.y = velocity(1) / velocity.head(2).norm();
            yaw.z = 0.0;

            if(use_last_yaw){
                cmd.yaw = last_yaw;
                yaw.x   = cos(last_yaw);
                yaw.y   = sin(last_yaw);
                yaw.z   = 0.0;
            }
            else if(pub_lead_yaw){
                if(use_lead_yaw_plan1){
                    /*  use the lead velocity direction  */
                    cmd.yaw = atan2(velocity_view(1), velocity_view(0));
                    cmd.yaw_dot = 1.0; 

                    yaw.x = velocity_view(0) / velocity_view.head(2).norm();
                    yaw.y = velocity_view(1) / velocity_view.head(2).norm();
                    yaw.z = 0.0;
                }
                else{
                    /*  use the position error direction  */
                    Vector3d pos_error = position_view - position;
                    cmd.yaw = atan2(pos_error(1), pos_error(0));
                    cmd.yaw_dot = 1.0; 
                    
                    yaw.x = pos_error(0) / pos_error.head(2).norm();
                    yaw.y = pos_error(1) / pos_error.head(2).norm();
                    yaw.z = 0.0;
                }
            }

            vis_pos.header.stamp = odom_time;
            vis_vel.header.stamp = odom_time;
            vis_acc.header.stamp = odom_time;
            vis_yaw.header.stamp = odom_time;

            vis_pos.points.clear();
            vis_vel.points.clear();
            vis_acc.points.clear();
            vis_yaw.points.clear();

            geometry_msgs::Point pt;
            pt.x = position(0);
            pt.y = position(1);
            pt.z = position(2);

            vis_pos.points.push_back(pt);
            vis_vel.points.push_back(pt);
            vis_acc.points.push_back(pt);
            vis_yaw.points.push_back(pt);

            pt.x = position(0) + velocity(0);
            pt.y = position(1) + velocity(1);
            pt.z = position(2) + velocity(2);
            vis_vel.points.push_back(pt);

            pt.x = position(0) + acceleration(0);
            pt.y = position(1) + acceleration(1);
            pt.z = position(2) + acceleration(2);
            vis_acc.points.push_back(pt);

            if(!pub_lead_yaw){
                pt.x = position(0) + velocity(0) / velocity.head(2).norm() * vis_yaw_length;
                pt.y = position(1) + velocity(1) / velocity.head(2).norm() * vis_yaw_length;
                pt.z = position(2);
                vis_yaw.points.push_back(pt);
            }
            else
            {   
                if(use_lead_yaw_plan1)
                {
                    pt.x = position(0) + velocity_view(0) / velocity_view.head(2).norm() * vis_yaw_length;
                    pt.y = position(1) + velocity_view(1) / velocity_view.head(2).norm() * vis_yaw_length;
                    pt.z = position(2);
                    vis_yaw.points.push_back(pt);
                }
                else
                {   
                    Vector3d pos_error = position_view - position;
                    pt.x = position(0) + pos_error(0) / pos_error.head(2).norm() * vis_yaw_length;
                    pt.y = position(1) + pos_error(1) / pos_error.head(2).norm() * vis_yaw_length;
                    pt.z = position(2);
                    vis_yaw.points.push_back(pt);
                }
            }

            _yaw_pub.publish(yaw);
        }

        // #4. just publish
        _cmd_pub.publish(cmd);
        last_yaw = cmd.yaw;
        /*cmd.position.x = odom.pose.pose.position.x;
        cmd.position.y = odom.pose.pose.position.y;
        cmd.position.z = odom.pose.pose.position.z;

        cmd.velocity.x = odom.twist.twist.linear.x;
        cmd.velocity.y = odom.twist.twist.linear.y;
        cmd.velocity.z = odom.twist.twist.linear.z;

        cmd.acceleration.x = odom.twist.twist.angular.x;
        cmd.acceleration.y = odom.twist.twist.angular.y;
        cmd.acceleration.z = odom.twist.twist.angular.z;

        cmd.yaw = atan2(2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y), 1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z));
        _current_pose_pub.publish(cmd);
*/
        _vis_pos_pub.publish(vis_pos);
        _vis_vel_pub.publish(vis_vel);
        _vis_acc_pub.publish(vis_acc);
        _vis_yaw_pub.publish(vis_yaw);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "time_optimal_trajectory_server");
    ros::NodeHandle handle("~");

    int _traj_order;
    double  _check_collision_time_interval, _time_commit, _time_horizon, _time_lead_yaw;
    bool _pub_lead_yaw, _use_lead_yaw_plan1;

    handle.param("local_planner/traj_order", _traj_order,  7);
    handle.param("local_planner/collision_check_timer", _check_collision_time_interval, 0.1);
    handle.param("local_planner/time_commit",   _time_commit,   0.5  );
    handle.param("local_planner/time_horizon",  _time_horizon,  4.0  );
    handle.param("local_planner/time_lead_yaw", _time_lead_yaw, 1.0  );
    handle.param("local_planner/pub_lead_yaw",       _pub_lead_yaw,       false);
    handle.param("local_planner/use_lead_yaw_plan1", _use_lead_yaw_plan1, false);

    TrajectoryServer server(handle, _traj_order, _check_collision_time_interval, _time_commit, _time_horizon, _time_lead_yaw, _pub_lead_yaw, _use_lead_yaw_plan1 );
    server.bezier_basis->setFixedOrder(_traj_order);

    ros::Rate rate(400);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
