//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

#define _TICK2ROSTIME(tick) (ros::Duration((double)(tick) / 400.0))

bool Aligner::acquire_stamp(ros::Time& msg_stamp, uint32_t tick) {
    ros::Time now_time = ros::Time::now();
    msg_stamp = ros::Time(0);

//    double dt = (now_time - (base_time + _TICK2ROSTIME(tick))).toSec();
//    printf("dt=%.3f\n", dt*1000.0);

    if (align_with_fmu) {
        if (align_state == State_t::unaligned) {
            base_time = now_time - _TICK2ROSTIME(tick);
            align_state = State_t::aligning;
            ROS_ERROR("[djiros] start imu align, base = %.3f, tick = %d", base_time.toSec(), tick);
            alignArray.clear();
            return false;
        }

        if (align_state == State_t::aligning) {
            align_state = State_t::aligned;
            ROS_ERROR("[djiros] ***** IMU ALIGNED *****");
            return false;
            // Here we do not do following tasks because I find that the dt varies largely,
            // from 1 ms to 10+ ms, such that I cannot test it.
            // Haven't figure out why it behaves like this.

            ROS_INFO_THROTTLE(1.0, "[djiros] IMU aliging...");
            if (alignArray.size() < ALIGN_BUFFER_SIZE) {
                alignArray.emplace_back(now_time, tick);
            } else {
                assert(alignArray.size() == ALIGN_BUFFER_SIZE);
                bool all_sample_pass_test = true;
                for (auto it = alignArray.begin(); it != alignArray.end(); ++it) {
                    double dt = (it->time - (base_time + _TICK2ROSTIME(it->tick))).toSec();
                    printf("dt=%.3f\n", dt*1000.0);
                    if (std::fabs(dt) > TIME_DIFF_CHECK) {
                        all_sample_pass_test = false;
                        alignArray.erase(alignArray.begin(), std::next(it));
                        break;
                    }
                }

                if (all_sample_pass_test) {
                    ROS_ERROR("[djiros] ***** IMU ALIGNED *****");
                    align_state = State_t::aligned;
                } else {
                    alignArray.emplace_back(now_time, tick);
                    base_time = now_time - _TICK2ROSTIME(alignArray.front().tick);
                }
            }
            return false;
        }

        if (align_state == State_t::aligned) {
            msg_stamp = base_time + _TICK2ROSTIME(tick);
            double dt = (now_time - msg_stamp).toSec();

            if (std::fabs(dt) > TIME_DIFF_ALERT) {
                static int cnt = 0;
                ++cnt;
                ROS_WARN_THROTTLE(
                        1.0, "[djiros] SysTime - TickTime = %.0f ms [%d]", dt * 1000, cnt);
            }

            last_msg_stamp = msg_stamp;
            return true;
        }
    } else  // Will not align with fmu, just use ros::Time::now()
    {
        ROS_DEBUG("[djiros] Not align with fmu, msg-seq-interval=%.3f",
                  (now_time - last_msg_stamp).toSec());
        msg_stamp = now_time;
        last_msg_stamp = msg_stamp;
        return true;
    }

    return false;
};
