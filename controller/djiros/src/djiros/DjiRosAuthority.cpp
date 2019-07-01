//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

void DjiRos::manually_leave_api_mode(bool need_release) {
  if (need_release) {
    obtain_control(false);
  }
  ROS_ERROR(ANSI_COLOR_CYAN
                "[djiros] Manually exit api mode."
                ANSI_COLOR_RESET);
}

void DjiRos::on_authority_ack(Vehicle *vehicle,
                              RecvContainer recvFrame,
                              DJI::OSDK::UserData userData) {
  DjiRos *p = (DjiRos *) userData;

  ACK::ErrorCode ack;

  ack.data = ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= (int) sizeof(uint16_t)) {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  } else {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ack.data == ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS) {
    ROS_ERROR(ANSI_COLOR_GREEN
                  "[djiros] ****** Acquire control success! ******"
                  ANSI_COLOR_RESET);
    p->sdk_control_flag = true;
  } else if (ack.data == ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS) {
    ROS_ERROR(ANSI_COLOR_CYAN
                  "[djiros] ****** Release control success! ******"
                  ANSI_COLOR_RESET);
    p->sdk_control_flag = false;
  } else if (p->api_trigger.getLevel() == 0) {
    // RC is not in F mode
    ROS_ERROR(ANSI_COLOR_CYAN
                  "[djiros] ****** Control is released by RC ******"
                  ANSI_COLOR_RESET);
    p->sdk_control_flag = false;
  } else if (ack.data == ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS){
    ACK::getErrorCodeMessage(ack, __func__);
    p->obtain_control(true);
  } else if (ack.data == ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_IN_PROGRESS){
    ACK::getErrorCodeMessage(ack, __func__);
    p->obtain_control(false);
  } else {
    ACK::getErrorCodeMessage(ack, __func__);
  }
}

void DjiRos::obtain_control(bool b) {
  if (b) {
    this->vehicle->obtainCtrlAuthority(DjiRos::on_authority_ack, this);
  } else {
    this->vehicle->releaseCtrlAuthority(DjiRos::on_authority_ack, this);
  }
}

