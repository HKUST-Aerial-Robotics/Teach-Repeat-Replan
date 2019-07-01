#include "MobileSample.h"

using namespace DJI;
using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;

void
parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                        DJI::OSDK::RecvContainer recvFrame,
                        DJI::OSDK::UserData      userData)
{

  uint16_t mobile_data_id;
  mobile_data_id =
    *(reinterpret_cast<uint16_t*>(&recvFrame.recvData.raw_ack_array));

  switch (mobile_data_id)
  {
    case 2:
      v->obtainCtrlAuthority(controlAuthorityMobileCallback);
      break;
    case 3:
      v->releaseCtrlAuthority(controlAuthorityMobileCallback);
      break;
    case 5:
      if(v->getFwVersion() != Version::M100_31)
      {
	v->control->action(Control::FlightCommand::startMotor,
        actionMobileCallback);
      }
      else
      {
        v->control->armMotors(actionMobileCallback);
      }
      break;
    case 6:
      if(v->getFwVersion() != Version::M100_31)
      {
        v->control->action(Control::FlightCommand::stopMotor,
        actionMobileCallback);
      }
      else
      {
	v->control->disArmMotors(actionMobileCallback);
      }
      break;
    default:
      break;
  }
}

void
controlAuthorityMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                               DJI::OSDK::UserData userData)
{
  ACK::ErrorCode ack;
  ack.data = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  unsigned char data    = 0x1;
  int           cbIndex = vehiclePtr->callbackIdIndex();

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }

  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                    OBTAIN_CONTROL_IN_PROGRESS)
  {
    vehiclePtr->obtainCtrlAuthority(controlAuthorityMobileCallback);
  }
  else if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                         RELEASE_CONTROL_IN_PROGRESS)
  {
    vehiclePtr->releaseCtrlAuthority(controlAuthorityMobileCallback);
  }
  else
  {
    // We have a success case.
    // Send this data to mobile
    AckReturnToMobile mobileAck;
    // Find out which was called: obtain or release
    if (recvFrame.recvInfo.buf[2] == ACK::OBTAIN_CONTROL)
    {
      mobileAck.cmdID = 0x02;
    }
    else if (recvFrame.recvInfo.buf[2] == ACK::RELEASE_CONTROL)
    {
      mobileAck.cmdID = 0x03;
    }
    mobileAck.ack = static_cast<uint16_t>(ack.data);
    vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                    sizeof(mobileAck));
  }
}

void
actionMobileCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                     DJI::OSDK::UserData userData)
{
  ACK::ErrorCode ack;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(uint16_t))
  {

    ack.info = recvFrame.recvInfo;

    if (vehiclePtr->getFwVersion() != Version::M100_31)
      ack.data = recvFrame.recvData.commandACK;
    else
      ack.data = recvFrame.recvData.ack;

    if (ACK::getError(ack))
      ACK::getErrorCodeMessage(ack, __func__);

    AckReturnToMobile mobileAck;
    const uint8_t     cmd[] = { recvFrame.recvInfo.cmd_set,
                            recvFrame.recvInfo.cmd_id };

    // startMotor supported in FW version >= 3.3
    // setArm supported only on Matrice 100
    if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::startMotor ||
        (memcmp(cmd, OpenProtocol::CMDSet::Control::setArm, sizeof(cmd)) &&
         recvFrame.recvInfo.buf[2] == true))
    {
      mobileAck.cmdID = 0x05;
      mobileAck.ack   = static_cast<uint16_t>(ack.data);
      vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                      sizeof(mobileAck));
    }
    else if (recvFrame.recvInfo.buf[2] == Control::FlightCommand::stopMotor ||
             (memcmp(cmd, OpenProtocol::CMDSet::Control::setArm, sizeof(cmd)) &&
              recvFrame.recvInfo.buf[2] == false))
    {
			mobileAck.cmdID = 0x06;
      mobileAck.ack   = static_cast<uint16_t>(ack.data);
      vehiclePtr->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&mobileAck),
                                      sizeof(mobileAck));
    }
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}
