/** @file dji_subscription.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Telemetry Subscription API for DJI OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_DATASUBSCRIPTION_H
#define DJI_DATASUBSCRIPTION_H

#include "dji_open_protocol.hpp"
#include "dji_telemetry.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief Package class to support Subscribe-style telemetry
 *
 *  @details Use the DJI_DataSubscription class to access telemetry.
 *
 *  @note This class is internal and does not need to be used by applications
 *  directly.
 */
class SubscriptionPackage
{
public:
#pragma pack(1)
  typedef struct PackageInfo
  {
    uint8_t  packageID;
    uint16_t freq;
    uint8_t  config;
    uint8_t  numberOfTopics;
  } PackageInfo;
#pragma pack()

public:
  SubscriptionPackage();
  ~SubscriptionPackage();

  void setPackageID(uint8_t id);
  void setConfig(uint8_t config);

  /*!
   * @brief Fill in necessary information for ADD_PACKAGE call
   * @param topics: List of TopicName
   * @param numberOfTopics: Number of topics to subscribe for this package
   * @param freq: Frequency of this package
   * @return
   */
  bool setTopicList(Telemetry::TopicName* topics, int numberOfTopics,
                    uint16_t freq);
  void allocateDataBuffer();
  void clearDataBuffer();

  void cleanUpPackage();

  /*!
   * @brief Serialize the info and uidList to a buffer to send to FC
   * @param buffer
   */
  int serializePackageInfo(uint8_t* buffer);

  void setUserUnpackCallback(VehicleCallBack userFunctionAfterPackageExtraction,
                             UserData        userData);

  bool isOccupied();

  void setOccupied(bool status);

  // Accessors to private variables:
  PackageInfo            getInfo();
  uint32_t*              getUidList(); // explicitly show it's a pointer
  Telemetry::TopicName*  getTopicList();
  uint32_t*              getOffsetList();
  uint8_t*               getDataBuffer();
  uint32_t               getBufferSize();
  VehicleCallBackHandler getUnpackHandler();

  /*!
  * @brief Helper function to do post processing when adding package is
  * successful.
  *
  */
  void packageAddSuccessHandler();

  /*!
  * @brief Helper function to do post processing when removing package is
  * successful.
  *
  */
  void packageRemoveSuccessHandler();

private: // Private variables
  bool        occupied;
  PackageInfo info;

  // We have only 30 topics and 5 packages.
  // So let's not bother with dynamic memory for now.
  // The UID
  uint32_t uidList[Telemetry::TOTAL_TOPIC_NUMBER];
  // The Name enum, used as index in the DataBase
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  // The offset of each topic in the data flow
  uint32_t offsetList[Telemetry::TOTAL_TOPIC_NUMBER];

  /*!
   * @brief The total size of all the actual topics in the package,
   *        not the size of the uidlist or namelist
   */
  uint32_t packageDataSize;

  /*!
   * @brief The buffer to hold data from FC
   */
  uint8_t* incomingDataBuffer;

  /*!
   * @brief Advanced users can optionally register a callback function
   *        (for each package) to run after every package is received.
   *        This function is called in the end of decodeCallback function.
   */
  VehicleCallBackHandler userUnpackHandler;
}; // class SubscriptionPackage

/*! @brief Telemetry API through asynchronous "Subscribe"-style messages
 *
 * @details The subscribe API allows fine-grained control over requesting
 * various topics at various frequencies.
 *
 * All topics at a certain frequency should be put into a single "package".
 *
 * @note Subscribe-style telemetry is a new feature, available since OSDK 3.3
 */
class DataSubscription
{
public: // public methods
  DataSubscription(Vehicle* vehicle);
  ~DataSubscription();

  Vehicle* getVehicle();
  /*!
   * @brief This is the interface for the end user to generate a package for
   * subscription.
   *
   * @param packageID: The ID of package it'll generate
   * @param numberOfTopics:
   * @param topicList: List of Topic Names to subscribe in the package
   * @param sendTimeStamp
   * @param freq
   * @return
   */
  bool initPackageFromTopicList(int packageID, int numberOfTopics,
                                Telemetry::TopicName* topicList,
                                bool sendTimeStamp, uint16_t freq);

  /*!
   * @brief Non-blocking call for version match
   */
  void verify();

  /*!
   * @brief Blocking call for version match.
   * @param timeout
   * @return
   */
  ACK::ErrorCode verify(int timeout); // blocking call

  /*!
  * @brief Non-blocking call for starting a package
  * @param packageID
  */
  void startPackage(int packageID);

  /*!
   * @brief Blocking call for start package
   * @param packageID
   * @param timeout
   * @return
   */
  ACK::ErrorCode startPackage(int packageID, int timeout); // blocking call

  void removePackage(int packageID);
  ACK::ErrorCode removePackage(int packageID, int timeout); // blocking call

  /*!
   * @brief Register a callback function after package[packageID] is received
   * @param packageID
   * @param userFunctionAfterPackageExtraction
   */
  void registerUserPackageUnpackCallback(
    int packageID, VehicleCallBack userFunctionAfterPackageExtraction,
    UserData userData = NULL);

  // Not implemented yet
  bool pausePackage(int packageID);
  bool resumePackage(int packageID);
  // bool changePackageFrequency(int packageID, uint16_t newFreq);

  /*!
   * @brief Callback function for non-blocking verify()
   *
   * @param API
   * @param header
   * @param userData
   */
  static void verifyCallback(Vehicle* vehiclePtr, RecvContainer rcvContainer,
                             UserData userData);

  static void addPackageCallback(Vehicle*      vehiclePtr,
                                 RecvContainer rcvContainer,
                                 UserData      pkgHandle);

  static void removePackageCallback(Vehicle*      vehiclePtr,
                                    RecvContainer rcvContainer,
                                    UserData      pkgHandle);

  /*!
   * @brief This callback function is called by recvReqData, case
   * CMD_ID_SUBSCRIBE.
   * @param API
   * @param header
   * @param subHandle: The pointer to the subscription object.
   */
  static void decodeCallback(Vehicle* vehiclePtr, RecvContainer rcvContainer,
                             UserData subscriptionPtr);

  template <Telemetry::TopicName           topic>
  typename Telemetry::TypeMap<topic>::type getValue()
  {
    typename Telemetry::TypeMap<topic>::type ans;

    void* p = Telemetry::TopicDataBase[topic].latest;

    protocol->getThreadHandle()->lockMSG();
    if (p)
    {
      ans = *reinterpret_cast<typename Telemetry::TypeMap<topic>::type*>(p);
      protocol->getThreadHandle()->freeMSG();
      return ans;
    }
    else
    {
      DERROR("Topic 0x%X value memory not initialized, return default", topic);
    }
    protocol->getThreadHandle()->freeMSG();

    memset(&ans, 0xFF, sizeof(ans));
    return ans;
  }

public: // public variables
  const static uint8_t   MAX_NUMBER_OF_PACKAGE = 5;
  VehicleCallBackHandler subscriptionDataDecodeHandler;

private: // private variables
  Vehicle*            vehicle;
  Protocol*           protocol;
  SubscriptionPackage package[MAX_NUMBER_OF_PACKAGE];

private: // private methods
  void extractOnePackage(RecvContainer*       pRcvContainer,
                         SubscriptionPackage* pkg);
};
}
}

#endif // DJI_DATASUBSCRIPTION_H
