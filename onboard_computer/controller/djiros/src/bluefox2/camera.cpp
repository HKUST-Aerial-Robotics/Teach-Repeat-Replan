#include "camera.h"
#include <boost/format.hpp>
#include <algorithm>

namespace bluefox2 {

Camera::Camera(ros::NodeHandle _param_nh)
    : pnode(_param_nh),
      m_fast_mode(false),
      m_hwsync_grab_count(0),
      m_max_req_number(-1),
      m_verbose_output(false),
      m_sync_offset_for_debug(0.0) {
    int exposure_time_us;
    bool use_color;
    bool use_hdr;
    bool has_hdr;
    bool use_binning;
    bool use_auto_exposure;
    int auto_speed;
    double fps;
    double gain;
    bool is_slave;

    int aec_desired_gray_value;
    int aec_control_delay_frame;

    pnode.param("use_color", use_color, false);
    pnode.param("use_hdr", use_hdr, true);
    pnode.param("has_hdr", has_hdr, true);
    pnode.param("use_binning", use_binning, false);
    pnode.param("use_auto_exposure", use_auto_exposure, false);
    pnode.param("aec_desired_gray_value", aec_desired_gray_value, 100);
    pnode.param("aec_control_delay_frame", aec_control_delay_frame, 0);
    pnode.param("exposure_time_us", exposure_time_us, 10000);
    pnode.param("auto_speed", auto_speed, 0);
    pnode.param("fps", fps, 30.0);
    pnode.param("gain", gain, 0.0);
    pnode.param("is_slave", is_slave, false);
    pnode.param("fast_mode", m_fast_mode, false);
    pnode.param("verbose_output", m_verbose_output, false);

    pnode.param("cam_cnt", cam_cnt, 1);

    m_fps = fps;

    ROS_ASSERT_MSG(cam_cnt <= MAX_CAM_CNT, "cam_cnt exceeds MAX_CAM_CNT(%d)", MAX_CAM_CNT);

    for (int i = 0; i < cam_cnt; i++) {
        std::string prefix = boost::str(boost::format("camera%d/") % i);

        CameraSetting cs;
        pnode.param(prefix + "serial", cs.serial, std::string("25001214"));
        pnode.param(prefix + "topic", cs.topic, boost::str(boost::format("image%d") % i));

        // pnode.param(prefix + std::string("fps"), fps, 30.0);
        pnode.param(prefix + "use_color", cs.use_color, use_color);
        pnode.param(prefix + "use_hdr", cs.use_hdr, use_hdr);
        pnode.param(prefix + "has_hdr", cs.has_hdr, has_hdr);
        pnode.param(prefix + "use_binning", cs.use_binning, use_binning);
        pnode.param(prefix + "use_auto_exposure", cs.use_auto_exposure, use_auto_exposure);
        pnode.param(
            prefix + "aec_desired_gray_value", cs.aec_desired_gray_value, aec_desired_gray_value);
        pnode.param(prefix + "aec_control_delay_frame",
                    cs.aec_control_delay_frame,
                    aec_control_delay_frame);
        pnode.param(prefix + "exposure_time_us", cs.exposure_time_us, exposure_time_us);
        pnode.param(prefix + "auto_speed", cs.auto_speed, auto_speed);
        pnode.param(prefix + "gain", cs.gain, gain);
        pnode.param(prefix + "is_slave", cs.is_slave, is_slave);

        ROS_ASSERT_MSG(cs.auto_speed < 3, "Invalid auto speed");
        ROS_ASSERT_MSG(!cs.serial.empty(), "Cannot read camera serial");

        camSettings[cs.serial] = cs;
    }

    std::pair<int, int> master_slave_cnt = {0, 0};
    for (auto& item : camSettings) {
        const CameraSetting& cs = item.second;
        img_publisher[cs.serial] = pnode.advertise<sensor_msgs::Image>(cs.topic, 10);
        img_buffer[cs.serial] = sensor_msgs::Image();

        if (cs.is_slave) {
            master_slave_cnt.second++;
        } else {
            master_slave_cnt.first++;
        }
    }

    if (master_slave_cnt.first == cam_cnt) {
        m_is_slave_mode = false;
    } else if (master_slave_cnt.second == cam_cnt) {
        m_is_slave_mode = true;
    } else {
        ROS_FATAL("This driver is designed for all master/all slave usage!");
        ROS_BREAK();
    }

    // Count cameras
    devCnt = devMgr.deviceCount();
    ROS_INFO("All %d mvBlueFOX devices", devCnt);

    // Init cameras
    ok = true;
    for (auto& item : camSettings) {
        bool current_camera_inited = false;
        CameraSetting& cs = item.second;
        for (unsigned int k = 0; k < devCnt; k++) {
            if (devMgr[k]->serial.read() == cs.serial) {
                ids[cs.serial] = k;
                if (!initSingleMVDevice(k, cs)) {
                    ok = false;
                    ROS_ERROR("Camera %s Init Failed.", cs.serial.c_str());
                    break;
                } else {
                    current_camera_inited = true;
                }
            }
        }
        if (!current_camera_inited) {
            ROS_ASSERT_MSG(false, "Camera %s not found!!!", cs.serial.c_str());
        }
    }

    // Subscribe for debug message
    m_offset_sub = pnode.subscribe<std_msgs::Float64>("sync_offset",
                                                             10,
                                                             boost::bind(&Camera::debug_sync_offset_callback, this, _1),
                                                             ros::VoidConstPtr(),
                                                             ros::TransportHints().tcpNoDelay());
}

Camera::~Camera() {
    for (auto& item : camSettings) {
        const int devIndex = ids.at(item.second.serial);
        fi[devIndex]->imageRequestReset(0, 0);
        devMgr[devIndex]->close();
    }
    ok = false;
}

bool Camera::is_slave_mode() const {
    return m_is_slave_mode;
}

bool Camera::is_fast_mode() const {
    return m_fast_mode;
}

bool Camera::isOK() {
    return ok;
}

bool Camera::initSingleMVDevice(unsigned int id, const CameraSetting& cs) {
    ROS_INFO("Camera Found:  %s(%s)",
             devMgr[id]->family.read().c_str(),
             devMgr[id]->serial.read().c_str());

    try {
        devMgr[id]->open();
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while opening the device " << devMgr[id]->serial.read()
                  << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
                  << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try {
        fi[id] = new mvIMPACT::acquire::FunctionInterface(devMgr[id]);
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while creating the function interface on device "
                  << devMgr[id]->serial.read() << "(error code: " << e.getErrorCode() << "("
                  << e.getErrorCodeAsString() << "))." << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try {
        statistics[id] = new mvIMPACT::acquire::Statistics(devMgr[id]);
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while initializing the statistical information on device "
                  << devMgr[id]->serial.read() << "(error code: " << e.getErrorCode() << "("
                  << e.getErrorCodeAsString() << "))." << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    // Set Properties
    mvIMPACT::acquire::SettingsBlueFOX settings(devMgr[id]);  // Using the "Base" settings (default)

    // Binning
    if (cs.use_binning) {
        settings.cameraSetting.binningMode.write(cbmBinningHV);
        ROS_INFO("2X Binning");
    } else {
        ROS_INFO("No Binning");
    }

    // Gain
    settings.cameraSetting.autoGainControl.write(agcOff);
    if (cs.gain >= 0.0) {
        settings.cameraSetting.gain_dB.write(cs.gain);
        ROS_INFO("Gain:  %f", settings.cameraSetting.gain_dB.read());
    } else {
        settings.cameraSetting.autoGainControl.write(agcOn);
        ROS_INFO("Auto Gain %d", settings.cameraSetting.autoGainControl.read());
    }

    // Auto exposure, modified controller for better results, be careful about the minimum exposure
    // time
    if (cs.use_auto_exposure) {
        // settings.cameraSetting.autoControlParameters.controllerSpeed.write(acsUserDefined);
        // settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
        // settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms.write(100);
        // settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms.write(0.0001);
        std::vector<mvIMPACT::acquire::TAutoControlSpeed> tacSpeedTable = {
            acsSlow, acsMedium, acsFast, acsUserDefined};
        settings.cameraSetting.autoControlParameters.controllerSpeed.write(
            tacSpeedTable.at(cs.auto_speed));
        settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(
            cs.aec_desired_gray_value);
        settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(
            cs.aec_control_delay_frame);
        settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(12);
        settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(cs.exposure_time_us);
        settings.cameraSetting.autoExposeControl.write(aecOn);
        ROS_INFO("Auto Exposure w/ Max Exposure Time (us) :  %d . Desired Grey Value: %d",
                 settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.read(),
                 settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.read());
    } else {
        settings.cameraSetting.autoExposeControl.write(aecOff);
        settings.cameraSetting.expose_us.write(cs.exposure_time_us);
        ROS_INFO("Exposure Time (us) :  %d", settings.cameraSetting.expose_us.read());
    }

    // HDR
    if (cs.has_hdr) {
        if (cs.use_hdr) {
            settings.cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
            settings.cameraSetting.getHDRControl().HDREnable.write(bTrue);
            ROS_INFO("Enable HDR ...");
            ROS_INFO("KneePoint 0:");
            ROS_INFO("  Voltage (mV):      %d",
                     settings.cameraSetting.getHDRControl()
                         .getHDRKneePoint(0)
                         .HDRControlVoltage_mV.read());
            ROS_INFO(
                "  Parts per Million: %d",
                settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRExposure_ppm.read());
            ROS_INFO("KneePoint 1:");
            ROS_INFO("  Voltage (mV):      %d",
                     settings.cameraSetting.getHDRControl()
                         .getHDRKneePoint(1)
                         .HDRControlVoltage_mV.read());
            ROS_INFO(
                "  Parts per Million: %d",
                settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRExposure_ppm.read());
        } else {
            settings.cameraSetting.getHDRControl().HDREnable.write(bFalse);
            ROS_INFO("HDR Off");
        }
    } else {
        ROS_INFO("No HDR");
    }

    // Color
    if (cs.use_color) {
        // RGB image
        settings.imageDestination.pixelFormat.write(idpfBGR888Packed);
        ROS_INFO("Color Images");
    } else {
        // Raw image
        settings.imageDestination.pixelFormat.write(idpfRaw);
        ROS_INFO("Grayscale/Bayer Images");
    }

    // prefill the capture queue. There can be more then 1 queue for some device, but only one for
    // now
    mvIMPACT::acquire::SystemSettings ss(devMgr[id]);
    ss.requestCount.write(4);

    // Trigger setting
    if (cs.is_slave) {
        ROS_INFO("Set Slave Mode\n");
        // settings.cameraSetting.triggerMode.write(ctmContinuous);
        settings.cameraSetting.triggerMode.write(ctmOnHighLevel);
        settings.cameraSetting.triggerSource.write(ctsDigIn0);
        settings.cameraSetting.frameDelay_us.write(0);
    } else {
        ROS_INFO("Set Master Mode\n");
        settings.cameraSetting.triggerMode.write(ctmOnDemand);
        settings.cameraSetting.flashMode.write(cfmDigout0);
        settings.cameraSetting.flashType.write(cftStandard);
        settings.cameraSetting.flashToExposeDelay_us.write(0);
    }

    return true;
}

void Camera::feedImages() {
    ROS_ASSERT(!is_slave_mode());
    ros::Rate r(m_fps);
    while (pnode.ok()) {
        send_software_request();
        if (grab_image_data()) {
            for (auto& item : img_publisher) {
                const std::string& serial = item.first;

                img_buffer.at(serial).header.stamp = capture_time;
                item.second.publish(img_buffer.at(serial));
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

void Camera::send_software_request() {
    // Request images from both cameras
    for (auto& item : ids) {
        const int devIndex = item.second;
        fi[devIndex]->imageRequestSingle();
    }
    usleep(10000);  // necessary short sleep to warm up the camera
    capture_time = ros::Time::now();
}

bool Camera::grab_image_data() {
    int requestNr[MAX_CAM_CNT] = {INVALID_ID};

    for (auto& item : ids) {
        ros::Time s1 = ros::Time::now();
        const int devIndex = item.second;
        requestNr[devIndex] = fi[devIndex]->imageRequestWaitFor(300);
        ros::Time s2 = ros::Time::now();
        ROS_INFO_COND(m_verbose_output, "imageRequestWaitFor %7.3f ms", (s2 - s1).toSec() * 1000.0);
    }

    bool status = true;
    for (auto& item : ids) {
        const int devIndex = item.second;
        if (!(fi[devIndex]->isRequestNrValid(requestNr[devIndex]))) {
            status = false;
            ROS_ERROR("[djiros/cam] Camera %s request timeout.", item.first.c_str());
        }
    }

    if (status) {
        int ok_cnt = 0;
        for (auto& item : ids) {
            const int devIndex = item.second;
            pRequest[devIndex] = fi[devIndex]->getRequest(requestNr[devIndex]);
            ok_cnt += pRequest[devIndex]->isOK();

            if (!pRequest[devIndex]->isOK()) {
                ROS_ERROR("[djiros/cam] Camera %s request failed.", item.first.c_str());
            }
        }

        if (ok_cnt == cam_cnt) {
            for (auto& item : ids) {
                const int devIndex = item.second;
                const std::string& serial = item.first;

                int Channels = pRequest[devIndex]->imageChannelCount.read();
                int Height = pRequest[devIndex]->imageHeight.read();
                int Width = pRequest[devIndex]->imageWidth.read();
                int Step = Width * Channels;
                std::string Encoding = Channels == 1 ? sensor_msgs::image_encodings::MONO8
                                                     : sensor_msgs::image_encodings::BGR8;

                img_buffer.at(serial).height = Height;
                img_buffer.at(serial).width = Width;
                img_buffer.at(serial).step = Step;
                img_buffer.at(serial).encoding = Encoding;

                ROS_ASSERT(static_cast<int>(Step * Height) == pRequest[devIndex]->imageSize.read());

                img_buffer.at(serial).data.resize(Step * Height);
                std::memcpy(&img_buffer.at(serial).data[0],
                            pRequest[devIndex]->imageData.read(),
                            pRequest[devIndex]->imageSize.read());

                img_buffer.at(serial).header.frame_id = boost::str(
                    boost::format("expose_us:%d") % pRequest[devIndex]->infoExposeTime_us.read());
            }

            // Release capture request
            for (auto& item : ids) {
                const int devIndex = item.second;
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }

            status = true;

        } else {
            ROS_ERROR("Not all requests are vaild, discard all data.");
            // Clear all image received and reset capture
            for (auto& item : ids) {
                const int devIndex = item.second;
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }

            status = false;
        }
    } else {
        ROS_ERROR("[djiros/cam] Timeout request/s.");
        // Clear all image received and reset capture
        for (auto& item : ids) {
            const int devIndex = item.second;
            if (fi[devIndex]->isRequestNrValid(requestNr[devIndex])) {
                pRequest[devIndex] = fi[devIndex]->getRequest(requestNr[devIndex]);
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }
        }
        status = false;
    }
    return status;
}

void Camera::send_hardware_request() {
    m_hwsync->req_queue.push(SyncReqInfo(SyncReqInfo::SingleRequestValue));
}

void Camera::send_driver_request() {
    // Request images from both cameras
    for (auto& item : ids) {
        const int devIndex = item.second;
        fi[devIndex]->imageRequestSingle();
    }
}

void Camera::reset_driver_request() {
    // Request images from both cameras
    for (auto& item : ids) {
        const int devIndex = item.second;
        fi[devIndex]->imageRequestReset(0, 0);
    }
}

bool Camera::wait_for_imu_ack(SyncAckInfo& sync_ack, int& queue_size) {
    ros::Time wait_start_time = ros::Time::now();
    while (1) {
        if (!pnode.ok()) {
            return false;
        }

        // Check imu response
        {
            std::lock_guard<std::mutex> lg(m_hwsync->ack_mutex);
            queue_size = m_hwsync->ack_queue.size();
            if (m_hwsync->ack_queue.size()) {
                if (m_hwsync->ack_queue.size() == 1) {
                    // ROS_INFO("ack queue size = %zu", m_hwsync->ack_queue.size());
                } else {
                    ROS_ERROR("ack queue size = %zu", m_hwsync->ack_queue.size());
                    ROS_ERROR(
                        "Cannot sync! Image capturing is too slow! Try to decrease fps/exposure or "
                        "turn off aec!");
                    //ros::shutdown();
                }
                // get first and erase it
                sync_ack = m_hwsync->ack_queue.front();
                m_hwsync->ack_queue.pop();

                // return the ack
                break;
            };
        }

        // sleep for the while loop
        ros::Duration(5.0 / 1000.0).sleep();

        ros::Duration dt = ros::Time::now() - wait_start_time;

        // Timeout set as two times longer than normal interval
        if (dt.toSec() > (1.0 / m_fps * 2)) {
            ROS_WARN_THROTTLE(1.0, "Wait %.3f secs for imu ack", dt.toSec());
            return false;
        }
    }

    return true;
}

void Camera::process_slow_sync() {
    ROS_ASSERT(is_slave_mode());
    ROS_ASSERT(m_hwsync.get());

    send_driver_request();

    ros::Rate r(m_fps);
    while (pnode.ok()) {
        send_hardware_request();

        // Wait for imu ack
        SyncAckInfo sync_ack;
        int queue_size = 0;
        if (!wait_for_imu_ack(sync_ack, queue_size)) {
            reset_driver_request();
            ROS_ERROR("reset all driver requests!");
            goto END_OF_OUT_LOOP;
        }

        // Imu ack received, verify it
        ROS_ASSERT(sync_ack.seq >= 0);

        // Get image from driver
        if (grab_image_data()) {
            ROS_INFO_COND(m_verbose_output, "Grab data with seq[%d]", sync_ack.seq);
            ROS_ASSERT_MSG(sync_ack.seq == m_hwsync_grab_count,
                           "sync_ack.seq=%d, hwsync_grab_count=%d",
                           sync_ack.seq,
                           m_hwsync_grab_count);

            for (auto& item : img_publisher) {
                const std::string& serial = item.first;

                capture_time = ros::Time(sync_ack.stamp);

                img_buffer.at(serial).header.stamp = capture_time;
                item.second.publish(img_buffer.at(serial));
            }
        }
        m_hwsync_grab_count++;  // inc it whether grab success or failed.
        send_driver_request();

    END_OF_OUT_LOOP:
        r.sleep();

        if (m_max_req_number > 0 && m_hwsync_grab_count >= m_max_req_number) {
            break;
        }
    }
}

void Camera::process_fast_sync() {
    ROS_ASSERT(is_slave_mode());
    ROS_ASSERT(is_fast_mode());
    ROS_ASSERT(m_hwsync.get());
    const int TRAIL_CNT = 5;
    ros::Time tm1, tm2;
    bool grab_result;

    ROS_INFO("[djiros/cam] Process several one-shot requests...");

    this->m_max_req_number = TRAIL_CNT;
    this->process_slow_sync();

    ROS_INFO("[djiros/cam] Clear buffers... (There may be request timeout)");

    for (size_t i = 0; i < 1; ++i) {
        grab_image_data();
        ros::Duration(1.0 / m_fps).sleep();
    }

    ROS_INFO("[djiros/cam] Start continuous requests");

    {
        std::lock_guard<std::mutex> lg(m_hwsync->req_mutex);
        m_hwsync->req_queue.push(SyncReqInfo(static_cast<int>(m_fps)));
    }
    m_hwsync_grab_count = 0;

    send_driver_request();

    ros::Rate r(200.0);
    while (pnode.ok()) {
        // Wait for imu ack
        SyncAckInfo sync_ack;
        int queue_size = 0;
        if (!wait_for_imu_ack(sync_ack, queue_size)) {
            ROS_ERROR("Wait failed.");
            goto END_OF_OUT_LOOP;
        }
        // Imu ack received, verify it
        ROS_ASSERT(sync_ack.seq >= 0);

        // Get image from driver
        tm1 = ros::Time::now();
        grab_result = grab_image_data();
        tm2 = ros::Time::now();
        ROS_INFO_COND(m_verbose_output, "grab time %.3f", (tm2 - tm1).toSec());

        ROS_INFO_COND(m_verbose_output,
                      "Try grabing data with imu_seq[%d] grab_seq[%d]",
                      sync_ack.seq,
                      m_hwsync_grab_count);

        if (grab_result) {
            for (auto& item : img_publisher) {
                const std::string& serial = item.first;

                capture_time = sync_ack.stamp;

                // After artificial calibrated, we found that actually image is taken one imu later
                // than triggered
                img_buffer.at(serial).header.stamp = capture_time + ros::Duration(m_sync_offset_for_debug);
                item.second.publish(img_buffer.at(serial));
            }
        }
        m_hwsync_grab_count++;  // inc it whether grab success or failed.
        tm1 = ros::Time::now();
        // if (queue_size > 0 && queue_size < 4) {
        // for(size_t k = 0; k < static_cast<size_t>(queue_size); ++k) {
        send_driver_request();
        // }
        // } else {
        // ROS_ERROR("[djiros/cam] queue_size = %d", queue_size);
        // }
        tm2 = ros::Time::now();
    // ROS_INFO("driver time %.3f", (tm2-tm1).toSec());

    END_OF_OUT_LOOP:
        // tm1 = ros::Time::now();
        r.sleep();
        // tm2 = ros::Time::now();
        // ROS_INFO("sleep time %.3f", (tm2-tm1).toSec());
    }
}

void Camera::debug_sync_offset_callback(const std_msgs::Float64ConstPtr& pMsg) {
    ROS_INFO("[djifox] Sync offset set to [img] = [imu] + %f", pMsg->data);
    m_sync_offset_for_debug = pMsg->data;
}

}  // End of namespace bluefox2
