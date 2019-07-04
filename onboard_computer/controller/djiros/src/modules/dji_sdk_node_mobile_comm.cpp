/** @file dji_sdk_node_mobile_comm.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the mobile communication functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void DJISDKNode::SDKfromMobileDataCallback(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData) {
  ((DJISDKNode*)userData)->fromMobileDataCallback(recvFrame);
}

void DJISDKNode::fromMobileDataCallback(RecvContainer recvFrame) {
  int dataLength = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 2;
  if (dataLength <= 100) {
    DSTATUS( "Received mobile Data of len %d\n", recvFrame.recvInfo.len);
    dji_sdk::MobileData mobile_data;
    mobile_data.data.resize(dataLength);
    for (int i=0; i<dataLength; i++)
    {
      mobile_data.data[i] = recvFrame.recvData.raw_ack_array[i];
    }
    from_mobile_data_publisher.publish(mobile_data);
  }
}

bool DJISDKNode::sendToMobileCallback(dji_sdk::SendMobileData::Request& request,
                                      dji_sdk::SendMobileData::Response& response){
  vehicle->moc->sendDataToMSDK(&request.data[0], request.data.size());
  response.result = true;
  return true;
}
