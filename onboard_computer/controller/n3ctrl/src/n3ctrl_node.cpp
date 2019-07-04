#include <ros/ros.h>
#include "N3CtrlFSM.h"

#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <signal.h>

#include <n3ctrl/ControllerDebug.h>
N3CtrlFSM* pFSM;

void mySigintHandler(int sig) {
    pFSM->stateVisualizer.publish_led_vis(ros::Time::now(), "null");
    ROS_ERROR("[N3Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "n3ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;

    Controller controller(param);
    HovThrKF hov_thr_kf(param);
    N3CtrlFSM fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    param.config_from_ros_handle(nh);
    param.init();
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);
    if (param.hover.set_hov_percent_to_zero) {
        // set to zero for debug
        fsm.hov_thr_kf.set_hov_thr(0.0);
    }
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    bool skip_wait_for_rc = false;
    if (param.work_mode.compare("simulation") == 0) {
        fsm.set_work_mode(N3CtrlFSM::SIMULATION);
        skip_wait_for_rc = true;
    } else if (param.work_mode.compare("sim_without_rc") == 0) {
        fsm.set_work_mode(N3CtrlFSM::SIM_WITHOUT_RC);
        skip_wait_for_rc = true;
    } else {
        fsm.set_work_mode(N3CtrlFSM::REALTIME);
    }

    if (param.js_ctrl_mode.compare("raw") == 0) {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_RAW);
    } else {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_FEEDBACK);
    }

    fsm.rc_data.set_default_mode(std::string("manual"));
    fsm.rc_data.set_default_mode(std::string("noapi"));
    fsm.controller.config();

    ros::Subscriber joy_sub =
        nh.subscribe<sensor_msgs::Joy>("joy",
                                       1000,
                                       boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         1000,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu",
                                       1000,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "cmd",
        1000,
        boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber idle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
        "idling",
        1000,
        boost::bind(&Idling_Data_t::feed, &fsm.idling_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber enter_js_sub =
        nh.subscribe<std_msgs::Header>("enter_js",
                                       1000,
                                       boost::bind(&Trigger_Data_t::feed, &fsm.trigger_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    fsm.controller.ctrl_pub = nh.advertise<sensor_msgs::Joy>("ctrl", 10);

    // fsm.controller.ctrl_so3_pub	=
    // 	nh.advertise<quadrotor_msgs::SO3Command>("ctrl_so3", 10);

    fsm.controller.ctrl_so3_attitude_pub =
        nh.advertise<geometry_msgs::QuaternionStamped>("ctrl_so3/attitude", 10);

    fsm.controller.ctrl_so3_thrust_pub =
        nh.advertise<geometry_msgs::WrenchStamped>("ctrl_so3/thrust", 10);

    fsm.controller.ctrl_vis_pub = nh.advertise<sensor_msgs::Imu>("ctrl_vis", 10);

    fsm.controller.ctrl_dbg_pub = nh.advertise<std_msgs::Header>("ctrl_dbg/info", 10);

    fsm.controller.ctrl_val_dbg_pub = nh.advertise<n3ctrl::ControllerDebug>("ctrl_dbg/value", 10);

    fsm.controller.ctrl_dbg_p_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/p", 10);

    fsm.controller.ctrl_dbg_v_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/v", 10);

    fsm.controller.ctrl_dbg_a_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/a", 10);

    fsm.controller.ctrl_dbg_att_des_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_des", 10);

    fsm.controller.ctrl_dbg_att_real_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_real", 10);

    fsm.hov_thr_kf.hov_thr_pub = nh.advertise<std_msgs::Float64>("hov_thr", 10);

    fsm.des_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("desire_pose", 10);

    fsm.fsm_dbg_pub = nh.advertise<std_msgs::Header>("fsm_dbg", 10);

    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);

    fsm.stateVisualizer.led_pub = nh.advertise<visualization_msgs::Marker>("state_led", 10);

    // essential for publishers and subscribers to get ready
    ros::Duration(0.5).sleep();

    if (skip_wait_for_rc) {
        ROS_INFO("[N3CTRL] Simulation, skip rc.");
    } else {
        ROS_INFO("[N3CTRL] Waiting for rc");
        while (ros::ok()) {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now())) {
                ROS_INFO("[N3CTRL] rc received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    // ros::Timer timer = nh.createTimer(ros::Duration(1.0/1000.0),
    // 	boost::bind(&N3CtrlFSM::process, &fsm, _1));

    ros::Rate r(2000.0);
    fsm.last_ctrl_time = ros::Time::now();
    // ---- process ----
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        fsm.process();
    }

    return 0;
}
