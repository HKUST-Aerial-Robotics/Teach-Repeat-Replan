//
// Created by ltb on 6/7/17.
//

/******** RC Map (M100) *********
*
*  +8000 <--->  0  <---> -8000
*   API  <---> ATT <--->  POS
*
*        CH3 +10000                     CH1 +10000
*               ^                              ^
*               |                              |                   / -10000
*    CH2        |                   CH0        |                  /
*  -10000 <-----------> +10000    -10000 <-----------> +10000    H
*               |                              |                  \
*               |                              |                   \ -4545
*               V                              V
*            -10000                         -10000
*
******** RC Map (A3) *********
*
*  -10000 <--->  0  <---> 10000
*   API  <---> ATT <--->  POS
*
*        CH3 +10000                     CH1 +10000
*               ^                              ^
*               |                              |                   / -5000
*    CH2        |                   CH0        |                  /
*  -10000 <-----------> +10000    -10000 <-----------> +10000    H
*               |                              |                  \
*               |                              |                   \ -10000
*               V                              V
*            -10000                         -10000
*
*
*   In this code, before publish, RC is transformed to M100 style to be compatible with controller
*
********** Frames **********
*
*       N(orth)                    U(p)                  F(orward)
*      /                           |                    /
*     /                            |    F(orward)      /
*    /______ E(ast)                |  /               /______ R(ight)
*    |                             | /                |
*    |                      ______ |/                 |
*    |D(own)               L(eft)                     |D(own)
*
*
****************************************/

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <string.h>

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <djiros/HardwareSync.h>
#include <dji_sdk/dji_sdk_node.h>

#define ASSERT_EQUALITY(x, y) ROS_ASSERT_MSG((x) == (y), "_1:%d _2:%d", (int)x, (int)y);

template <class T>
class LevelTrigger {
public:
    LevelTrigger() : LevelTrigger(static_cast<T>(0)){};
    LevelTrigger(T thres_)
            : threshold(thres_), has_last_value(false), raise_trigger(false), drop_trigger(false){};

    T getValue() {
        return value;
    };

    bool getLevel() {
        return value > threshold;
    };

    // Will read and clear "Raise Trigger"
    bool isRaiseEdge() {
        if (raise_trigger) {
            raise_trigger = false;
            return true;
        } else {
            return false;
        }
    };

    // Will read and clear "Drop Trigger"
    bool isDropEdge() {
        if (drop_trigger) {
            drop_trigger = false;
            return true;
        } else {
            return false;
        }
    };

    void setValue(T x) {
        value = x;

        if (!has_last_value) {
            has_last_value = true;
            last_value = value;
            return;
        }

        if (last_value <= threshold && threshold < value) {
            raise_trigger = true;
        }

        if (last_value > threshold && threshold >= value) {
            drop_trigger = true;
        }

        last_value = value;
    };

    void clear() {
        has_last_value = false;
        raise_trigger = false;
        drop_trigger = false;
    }

private:
    const T threshold;
    bool has_last_value;

    T value;
    T last_value;

    bool raise_trigger;
    bool drop_trigger;
};

class Aligner {
public:
    // align related variables
    enum struct State_t { unaligned, aligning, aligned };

    State_t align_state;
    bool align_with_fmu;

    Aligner():align_state(State_t::unaligned), align_with_fmu(false){};

    // If no need for align, set msg_stamp and return true;
    // If need align, return false when ualigned/aligning, and return true & set msg_stamp when aligned.
    bool acquire_stamp(ros::Time& msg_stamp, uint32_t tick);
    ros::Time acquire_latest_stamp() {return last_msg_stamp;};

private:
    struct AlignData_t {
        ros::Time time;
        uint32_t tick;
        AlignData_t(ros::Time _time, uint32_t _tick) : time(_time), tick(_tick){};
    };

    ros::Time last_msg_stamp;
    ros::Time base_time;
    std::list<AlignData_t> alignArray;
    static int constexpr ALIGN_BUFFER_SIZE = 100;
    static double constexpr TIME_DIFF_CHECK = 0.010;
    static double constexpr TIME_DIFF_ALERT = 0.02050;
};

class DjiRos : public DJISDKNode {
public:
    DjiRos(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~DjiRos();

    // ---- Data subscription / publish related functions ----
    bool initPublisher(ros::NodeHandle &nh); // overloaded

    bool initDataSubscribeFromFC(); // overloaded

    // Overloaded callbacks for subscription data
    static void onReceive50HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                  DJI::OSDK::UserData userData); // overloaded

    static void onReceive400HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                   DJI::OSDK::UserData userData); // overloaded


    // ---- Functional ----
    void process();

    Aligner aligner;
    LevelTrigger<int16_t> api_trigger;

    // ---- Authority related ----
    /* clang-format off */
    // Obtain and release control status transfer instructions
    // released             + <switch info F mode>                  = wait_for_command
    // wait_for_command     + <control command is streaming in>     = obtaining
    // obtaining            + <sdk response obtain successfully>    = obtained
    // obtained             + <control command stream timeout>      = released
    // wait_for_command     + <control command wait timeout>        = released
    /* clang-format on */
    enum struct CtrlState_t { released, wait_for_command, obtaining, obtained };
    CtrlState_t ctrl_state;
    double ctrl_cmd_wait_timeout_limit;
    double ctrl_cmd_stream_timeout_limit;
    ros::Time last_ctrl_stamp;
    ros::Time wait_start_stamp;
    bool sdk_control_flag;

    ros::Subscriber ctrl_sub;
    bool initSubscriber(ros::NodeHandle &nh); // overloaded
    void control_callback(const sensor_msgs::JoyConstPtr& pMsg);

    bool isTakeOff = false;
    bool isEmergency = false;
    // extern bool isn1ctrlReady = false;
    ros::ServiceServer drone_arm_server;
    bool initServices(ros::NodeHandle &nh);
    bool droneArmCallback(dji_sdk::DroneArmControl::Request&  request, dji_sdk::DroneArmControl::Response& response);    
    bool sdkCtrlAuthorityCallback(dji_sdk::SDKControlAuthority::Request&  request, dji_sdk::SDKControlAuthority::Response& response);
    static void on_authority_ack(Vehicle* vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData);
    void obtain_control(bool b);
    void manually_leave_api_mode(bool need_release);

    bool m_verbose_output;

public:
    std::shared_ptr<HardwareSynchronizer> m_hwsync;
    int m_hwsync_ack_count;

private:
    // parameters
    Eigen::Matrix3d ros_R_fc;
    double gravity; // multiplied to IMU
    bool sensor_mode; // only publish or publish + control
};
