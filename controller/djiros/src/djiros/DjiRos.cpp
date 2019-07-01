//
// Created by ltb on 6/7/17.
//

#include <dji_sdk/dji_sdk_node.h>
#include <djiros/DjiRos.h>

using namespace DJI::OSDK;

DjiRos::DjiRos(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : DJISDKNode(),
      api_trigger(static_cast<int16_t>(5000)), // 8000 is F-mode, 0 is A-mode, set threshold to 5000
      ctrl_state(CtrlState_t::released),
      ctrl_cmd_wait_timeout_limit(0.0),
      ctrl_cmd_stream_timeout_limit(0.0),
      last_ctrl_stamp(ros::Time(0)),
      sdk_control_flag(false),
      m_verbose_output(false),
      m_hwsync_ack_count(0) {
  nh_private.param("sensor_mode", sensor_mode, false); // only publish or publish + control
  nh_private.param("serial_name",   serial_device, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate",     baud_rate, 921600);
  nh_private.param("app_id",        app_id,    123456);
  nh_private.param("app_version",   app_version, 1);
  nh_private.param("enc_key",       enc_key, std::string("abcd1234"));
  nh_private.param("uart_or_usb",   uart_or_usb, 0); // chosse uart as default
  nh_private.param("drone_version", drone_version, std::string("M100")); // choose M100 as default
  nh_private.param("gravity_const", gravity_const, 9.801);
  nh_private.param("align_time",    align_time_with_FC, true);
  nh_private.param("use_broadcast", user_select_BC, false);

  // SDK initialization
  if (!initVehicle(nh_private))
  {
    ROS_ERROR("Vehicle initialization failed");
    throw std::runtime_error("initVehicle failed");
  }

  else
  {
    if (!sensor_mode) {
      if (!initSubscriber(nh_private)) {
        throw std::runtime_error("initSubscriber failed");
      }
    }

    if (!initPublisher(nh_private)) {
      throw std::runtime_error("initPublisher failed");
    }

    if(!initServices(nh_private)) {
      throw std::runtime_error("initServiceclient failed");
    }
    
  }

  // Ros related initialization
  nh_private.param("align_with_fmu",
                   aligner.align_with_fmu,
                   true);  // Whether djiros will use ticks from fmu to align with it
  nh_private.param("gravity", gravity, 9.79);
  nh.param("ctrl_cmd_stream_timeout", ctrl_cmd_stream_timeout_limit, 0.1);
  nh.param("ctrl_cmd_wait_timeout", ctrl_cmd_wait_timeout_limit, 3.0);

  ros_R_fc << 1, 0, 0, 0, -1, 0, 0, 0, -1;

  ROS_INFO("[DjiRos] ==== Initialization finished ====");
}

DjiRos::~DjiRos() {
  if (!sensor_mode) {
    ROS_INFO("[djiros] Release on exit.");
    obtain_control(false); // release control authority
    vehicle->hardSync->setSyncFreq(0, 0); // stop sync trigger

    // Wait sometime for display information, but will shutdown even if sdk has wrong/no
    // response
    ros::Duration(0.5).sleep();
  }
}

void DjiRos::process() {
  ros::Time now_time = ros::Time::now();

  // Hardware sync processing
  if (m_hwsync.get()) {
    // read queue and send request to API
    std::lock_guard<std::mutex> lg(m_hwsync->req_mutex);

    while (m_hwsync->req_queue.size()) {  // There are requests in the queue
      SyncReqInfo sync_req_info = m_hwsync->req_queue.front();
      m_hwsync->req_queue.pop();

      ROS_ASSERT(sync_req_info.freq >= 0);
      vehicle->hardSync->setSyncFreq(sync_req_info.freq, 0);

      if (sync_req_info.freq > 0) {
        // clear counter
        m_hwsync_ack_count = 0;
        ROS_INFO("[djiros] Set sync freq to %d hz", sync_req_info.freq);
      }
    }
  }


  // Control authority state machine processing
  if (!sensor_mode) {
    bool switch_into_F_mode = api_trigger.isRaiseEdge();
    bool out_of_F_mode = (api_trigger.getLevel() == 0);
    // This timeout checks if control command stops streaming in
    bool ctrl_cmd_stream_timeout =
        (now_time - last_ctrl_stamp).toSec() > ctrl_cmd_stream_timeout_limit;
    // This timeout checks if it waits too long for the command
    bool ctrl_cmd_wait_timeout =
        (now_time - wait_start_stamp).toSec() > ctrl_cmd_wait_timeout_limit;
    bool ctrl_obtained = (sdk_control_flag == 1);

    // ROS_INFO_STREAM("bools: " << switch_into_F_mode <<
    // out_of_F_mode <<
    // ctrl_cmd_stream_timeout <<
    // ctrl_cmd_wait_timeout <<
    // ctrl_obtained);

    if (ctrl_state == CtrlState_t::released) {
      if (switch_into_F_mode) {
        ctrl_state = CtrlState_t::wait_for_command;
        wait_start_stamp = now_time;
        ROS_INFO("[djiros] Switch into API by RC");
      }
    } else if (ctrl_state == CtrlState_t::wait_for_command) {
      if (out_of_F_mode) {
        manually_leave_api_mode(false);
        ctrl_state = CtrlState_t::released;
      } else {
        if (!ctrl_cmd_stream_timeout) {
          ctrl_state = CtrlState_t::obtaining;
          obtain_control(true);
          ROS_INFO("[djiros] Command received, obtaining authority");
        } else {
          if (ctrl_cmd_wait_timeout) {
            ctrl_state = CtrlState_t::released;
            ROS_WARN("[djiros] No ctrl cmd received. Exit api mode.");
          } else {
            if ((now_time - wait_start_stamp).toSec() > 1.0) {
              ROS_INFO_THROTTLE(1.0, "[djiros] Waiting for control command ...");
            }
          }
        }
      }
    } else if (ctrl_state == CtrlState_t::obtaining) {
      if (out_of_F_mode) {
        manually_leave_api_mode(true);
        ctrl_state = CtrlState_t::released;
      } else if (ctrl_obtained) {
        ctrl_state = CtrlState_t::obtained;
      } else {
        ROS_INFO_DELAYED_THROTTLE(1.0, "[djiros] Obtaining control...");
      }
    } else if (ctrl_state == CtrlState_t::obtained) {
      if (out_of_F_mode) {
        manually_leave_api_mode(true);
        ctrl_state = CtrlState_t::released;
      } else {
        if (ctrl_cmd_stream_timeout) {
          ROS_ERROR("[djiros] Control command is stopped for [%.0f] ms! Exit api mode.",
                    (now_time - last_ctrl_stamp).toSec() * 1000.0);
          obtain_control(false);
          ctrl_state = CtrlState_t::released;
        }
      }
    }
  }

  return;
}
