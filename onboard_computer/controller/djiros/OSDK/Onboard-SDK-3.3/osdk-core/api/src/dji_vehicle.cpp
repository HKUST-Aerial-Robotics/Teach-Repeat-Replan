/** @file dji_vehicle.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Vehicle API for DJI onboardSDK library
 *
 *  @copyright 2017 DJI. All right reserved.
 *
 */

#include "dji_vehicle.hpp"
#include <new>

using namespace DJI;
using namespace DJI::OSDK;

Vehicle::Vehicle(const char* device, uint32_t baudRate, bool threadSupport)
  : protocolLayer(NULL)
  , subscribe(NULL)
  , broadcast(NULL)
  , control(NULL)
  , camera(NULL)
  , gimbal(NULL)
  , mfio(NULL)
  , moc(NULL)
  , missionManager(NULL)
  , hardSync(NULL)
  , readThread(NULL)
  , callbackThread(NULL)
{
  if (!device)
    DERROR("Illegal serial device handle!\n");

  this->threadSupported = threadSupport;
  this->device          = device;
  this->baudRate        = baudRate;
  callbackId            = 0;
  ackErrorCode.data     = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  mandatorySetUp();
  functionalSetUp();
}

Vehicle::Vehicle(bool threadSupport)
  : protocolLayer(NULL)
  , subscribe(NULL)
  , broadcast(NULL)
  , control(NULL)
  , camera(NULL)
  , mfio(NULL)
  , moc(NULL)
  , missionManager(NULL)
  , hardSync(NULL)
  , readThread(NULL)
  , callbackThread(NULL)
{
  this->threadSupported = threadSupport;
  callbackId            = 0;

  mandatorySetUp();
}

void
Vehicle::mandatorySetUp()
{
  /*
   * Initialize buffers for threaded callbacks
   */
  if (this->threadSupported)
  {
    // We only need a buffer of recvContainers if we are using threads
    this->nbCallbackRecvContainer = new RecvContainer[200];
    this->circularBuffer = new CircularBuffer();
  }

  /*
   * @note Initialize predefined callbacks
   */
  initCallbacks();

  /*
   * @note Initialize CMD_SET support matrix to identify
   * CMD_SET availability for paritcular FW version
   */
  initCMD_SetSupportMatrix();

  /*
   * @note Initialize communication layer
   */
  if (!initOpenProtocol())
  {
    DERROR("Failed to initialize Protocol Layer!\n");
  }

  /*
   * @note Initialize read thread
   */
  if (!initPlatformSupport())
  {
    DERROR("Failed to initialize platform support!\n");
  }
}


void
Vehicle::functionalSetUp()
{
  if (!initVersion())
  {
    DERROR("Failed to initialize Version! Please exit.\n");
    return;
  }
  else if(this->getFwVersion() < extendedVersionBase &&
      this->getFwVersion() != Version::M100_31)
  {
    DERROR("Upgrade firmware using Assistant software!\n");
    return;
  }

  /*
   * Initialize subscriber if supported
   */
  if (!initSubscriber())
  {
    DSTATUS("Subscriber not supported!\n");
  }

  /*
   * Initialize broadcast if supported
   */
  if (!initBroadcast())
  {
    DSTATUS("Broadcast not supported!\n");
  }

  /*
   * @note Initialize Movement Control
   */
  if (!initControl())
  {
    DERROR("Control not supported!\n");
  }

  /*
   * @note Initialize external components
   * like Camera and MFIO
   */

  if (!initCamera())
  {
    DERROR("Failed to initialize Camera!\n");
  }

  if (!initGimbal())
  {
    DERROR("Failed to initialize Gimbal!\n");
  }
  /*
   * Initialize MFIO if supported
   */
  if (!initMFIO())
  {
    DSTATUS("MFIO not supported!\n");
  }

  /*
   * Initialize Mobile-Onboard Communication (MobileCommunication)
   */
  if (!initMOC())
  {
    DERROR("Failed to initialize MobileCommunication!\n");
  }

  if (!initMissionManager())
  {
    DERROR("Failed to initialize Mission Manager!\n");
  }

  if (!initHardSync())
  {
    DERROR("Hardware Sync not supported!\n");
  }

  if (!initVirtualRC())
  {
    DERROR("Virtual RC not supported!\n");
  }
}

void
Vehicle::initCMD_SetSupportMatrix()
{
  cmd_setSupportMatrix[0].cmdSet    = OpenProtocol::CMDSet::activation;
  cmd_setSupportMatrix[0].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[1].cmdSet    = OpenProtocol::CMDSet::control;
  cmd_setSupportMatrix[1].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[2].cmdSet    = OpenProtocol::CMDSet::broadcast;
  cmd_setSupportMatrix[2].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[3].cmdSet    = OpenProtocol::CMDSet::mission;
  cmd_setSupportMatrix[3].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[4].cmdSet    = OpenProtocol::CMDSet::hardwareSync;
  cmd_setSupportMatrix[4].fwVersion = extendedVersionBase;

  // Not supported in extendedVersionBase
  cmd_setSupportMatrix[5].cmdSet    = OpenProtocol::CMDSet::virtualRC;
  cmd_setSupportMatrix[5].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[7].cmdSet    = OpenProtocol::CMDSet::mfio;
  cmd_setSupportMatrix[7].fwVersion = extendedVersionBase;

  cmd_setSupportMatrix[8].cmdSet    = OpenProtocol::CMDSet::subscribe;
  cmd_setSupportMatrix[8].fwVersion = extendedVersionBase;
}

void
Vehicle::callbackPoll()
{
  VehicleCallBackHandler cbVal;
  RecvContainer          recvCont;
  //! If Head = Tail, there is no data in the buffer, do not call cbPop.
  protocolLayer->getThreadHandle()->lockNonBlockCBAck();
  if (this->circularBuffer->head != this->circularBuffer->tail)
  {
    circularBuffer->cbPop(circularBuffer, &cbVal, &recvCont);
    protocolLayer->getThreadHandle()->freeNonBlockCBAck();
    cbVal.callback(this, recvCont, cbVal.userData);
  }
  else
  {
    protocolLayer->getThreadHandle()->freeNonBlockCBAck();
  }
}

Vehicle::~Vehicle()
{
  if (threadSupported)
  {
    this->readThread->stopThread();
    this->callbackThread->stopThread();
    delete[](nbCallbackRecvContainer);
  }
  delete this->camera;
  delete this->gimbal;
  delete this->control;
  if (this->mfio)
    delete this->mfio;
  if (this->moc)
    delete this->moc;
  if (this->broadcast)
    delete this->broadcast;
  if (this->subscribe)
  {
    delete this->subscribe;
    this->subscribe = NULL;
  }
  if (hardSync)
    delete this->hardSync;
  delete this->missionManager;
  delete this->protocolLayer;
  if (threadSupported)
    delete this->readThread;
}

bool
Vehicle::initOpenProtocol()
{
  this->protocolLayer =
    new (std::nothrow) Protocol(this->device, this->baudRate);
  if (this->protocolLayer == 0)
  {
    return false;
  }

  return true;
}


bool
Vehicle::initPlatformSupport()
{
#ifdef QT
  if (threadSupported)
  {
    OSDKThread* readThreadPtr = new (std::nothrow) OSDKThread(this, 2);
    if (readThreadPtr == 0)
    {
      DERROR("Failed to initialize read thread!\n");
    }
    else
    {
      QThread* qReadThread = new QThread;
      readThreadPtr->setQThreadPtr(qReadThread);
      readThreadPtr->moveToThread(qReadThread);
      QObject::connect(qReadThread, SIGNAL(started()), readThreadPtr, SLOT(run()));
      QObject::connect(qReadThread, SIGNAL(finished()), qReadThread, SLOT(deleteLater()));
      qReadThread->start();
      this->readThread = readThreadPtr;
    }

    OSDKThread* cbThreadPtr = new (std::nothrow) OSDKThread(this, 3);
    if (cbThreadPtr == 0)
    {
      DERROR("Failed to initialize callback thread!\n");
    }
    else
    {
      QThread* qCbThread = new QThread;
      cbThreadPtr->setQThreadPtr(qCbThread);
      cbThreadPtr->moveToThread(qCbThread);
      QObject::connect(qCbThread, SIGNAL(started()), cbThreadPtr, SLOT(run()));
      QObject::connect(qCbThread, SIGNAL(finished()), qCbThread, SLOT(deleteLater()));
      qCbThread->start();
      this->callbackThread = cbThreadPtr;
    }
  }
#elif STM32
  //! Threads not supported by default
  this->readThread = NULL;
  return true;
#elif defined(__linux__)
  if (threadSupported)
  {
    this->callbackThread = new (std::nothrow) PosixThread(this, 3);
    if (this->callbackThread == 0)
    {
      DERROR("Failed to initialize read callback thread!\n");
    }

    this->readThread = new (std::nothrow) PosixThread(this, 2);
    if (this->readThread == 0)
    {
      DERROR("Failed to initialize read thread!\n");
    }
  }
#endif
  bool readThreadStatus = readThread->createThread();
  bool cbThreadStatus   = callbackThread->createThread();
  return (readThreadStatus && cbThreadStatus);
}

bool
Vehicle::initVersion()
{
#if STM32
  //! Non blocking call for STM32 as it does not support multi-thread
  getDroneVersion();
  STM32F4::delay_nms(2000);
#elif defined(QT)
  //! Non-blocking call for QT sample, thread sync not supported yet
  getDroneVersion();
  QThread::msleep(200);

#else
  ACK::DroneVersion ack = getDroneVersion(wait_timeout);
#endif
  if (this->getFwVersion() == 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool
Vehicle::parseDroneVersionInfo(Version::VersionData& versionData,
                               uint8_t*              ackPtr)
{

  Version::VersionData versionStruct;

  //! Note down our starting point as a sanity check
  uint8_t* startPtr = ackPtr;
  //! 2b ACK.
  versionStruct.version_ack = ackPtr[0] + (ackPtr[1] << 8);
  ackPtr += 2;

  //! Next, we might have CRC or ID; Put them into a variable that we will parse
  //! later. Find next \0
  uint8_t crc_id[16] = {};
  int     i          = 0;
  while (*ackPtr != '\0')
  {
    crc_id[i] = *ackPtr;
    i++;
    ackPtr++;
    if (ackPtr - startPtr > 18)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }
  //! Fill in the termination character
  crc_id[i] = *ackPtr;
  ackPtr++;

  //! Now we're at the name. First, let's fill up the name field.
  memcpy(versionStruct.version_name, ackPtr, 32);

  //! Now, we start parsing the name. Let's find the second space character.
  while (*ackPtr != ' ')
  {
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  } //! Found first space ("SDK-v1.x")
  ackPtr++;

  while (*ackPtr != ' ')
  {
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  } //! Found second space ("BETA")
  ackPtr++;

  //! Next is the HW version
  int j = 0;
  while (*ackPtr != '-')
  {
    versionStruct.hwVersion[j] = *ackPtr;
    ackPtr++;
    j++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }
  //! Fill in the termination character
  versionStruct.hwVersion[j] = '\0';
  ackPtr++;

  //! Finally, we come to the FW version. We don't know if each clause is 2 or 3
  //! digits long.
  int ver1 = 0, ver2 = 0, ver3 = 0, ver4 = 0;

  while (*ackPtr != '.')
  {
    ver1 = (*ackPtr - 48) + 10 * ver1;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver2 = (*ackPtr - 48) + 10 * ver2;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver3 = (*ackPtr - 48) + 10 * ver3;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '\0')
  {
    ver4 = (*ackPtr - 48) + 10 * ver4;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      DERROR("Drone version was not obtained. Please restart the program or "
             "call getDroneVersion\n");
      return false;
    }
  }

  versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);

  //! Special cases
  //! M100:
  if (strcmp(versionStruct.hwVersion, "M100") == 0)
  {
    //! Bug in M100 does not report the right FW.
    ver3                    = 10 * ver3;
    versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);
  }
  //! M600/A3 FW 3.2.10
  if (versionStruct.fwVersion == Version::FW(3, 2, 10, 0))
  {
    //! Bug in M600 does not report the right FW.
    ver3                    = 10 * ver3;
    versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);
  }

  //! Now, we can parse the CRC and ID based on FW version. If it's older than
  //! 3.2 then it'll have a CRC, else not.
  if (versionStruct.fwVersion < Version::FW(3, 2, 0, 0))
  {
    versionStruct.version_crc =
      crc_id[0] + (crc_id[1] << 8) + (crc_id[2] << 16) + (crc_id[3] << 24);
    uint8_t* id_ptr = &crc_id[4];

    int i = 0;
    while (*id_ptr != '\0')
    {
      versionStruct.hw_serial_num[i] = *id_ptr;
      i++;
      id_ptr++;
      if (id_ptr - &crc_id[4] > 12)
      {
        DERROR("Drone ID was not obtained.\n");
        return false; //! Not catastrophic error
      }
    }
    //! Fill in the termination character
    versionStruct.hw_serial_num[i] = *id_ptr;
  }
  else
  {
    versionStruct.version_crc = 0;
    uint8_t* id_ptr           = &crc_id[0];

    int i = 0;
    while (*id_ptr != '\0')
    {
      versionStruct.hw_serial_num[i] = *id_ptr;
      i++;
      id_ptr++;
      if (id_ptr - &crc_id[0] > 16)
      {
        DERROR("Drone ID was not obtained.\n");
        return false; //! Not catastrophic error
      }
    }
    //! Fill in the termination character
    versionStruct.hw_serial_num[i] = *id_ptr;
  }

  //! Finally, we print stuff out.

  if (versionStruct.fwVersion > Version::FW(3, 1, 0, 0))
  {
    DSTATUS("Device Serial No. = %.16s\n", versionStruct.hw_serial_num);
  }
  DSTATUS("Hardware = %.12s\n", versionStruct.hwVersion);
  DSTATUS("Firmware = %d.%d.%d.%d\n", ver1, ver2, ver3, ver4);
  if (versionStruct.fwVersion < Version::FW(3, 2, 0, 0))
  {
    DSTATUS("Version CRC = 0x%X\n", versionStruct.version_crc);
  }

  versionData = versionStruct;
  return true;
}

void
Vehicle::initCallbacks()
{
  hotPointCallback.callback = 0;
  wayPointCallback.callback = 0;
  hotPointCallback.userData = 0;
  wayPointCallback.userData = 0;
  missionCallback.callback  = 0;
  missionCallback.userData  = 0;
}

bool
Vehicle::initSubscriber()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::subscribe))
  {
    this->subscribe = new DataSubscription(this);
    if (this->subscribe == 0)
    {
      DERROR("Failed to allocate memory for Subscriber!\n");
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool
Vehicle::initBroadcast()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::broadcast))
  {
    this->broadcast = new (std::nothrow) DataBroadcast(this);
    if (this->broadcast == 0)
    {
      DERROR("Failed to allocate memory for Broadcast!\n");
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool
Vehicle::initControl()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::control))
  {
    this->control = new (std::nothrow) Control(this);
    if (this->control == 0)
    {
      DERROR("Failed to allocate memory for Control!\n");
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool
Vehicle::initCamera()
{
  this->camera = new (std::nothrow) Camera(this);

  if (this->camera == 0)
  {
    DERROR("Failed to allocate memory for Camera!\n");
    return false;
  }
  return true;
}

bool
Vehicle::initGimbal()
{
  // Gimbal information via subscription
  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_STATUS>::type
      subscriptionGimbal;

  if(this->getFwVersion() != Version::M100_31)
  {
    ACK::ErrorCode ack = this->subscribe->verify(wait_timeout);
    if(ACK::getError(ack))
    {
      DERROR("Failed to verify subscription!\n");
      return false;
    }

    Telemetry::TopicName topicList0[] = { Telemetry::TOPIC_GIMBAL_STATUS };
    int nTopic0 = sizeof(topicList0) / sizeof(topicList0[0]);

    bool result =
      this->subscribe->initPackageFromTopicList(0, nTopic0, topicList0, 0, 50);
    if (result)
    {
      ack = this->subscribe->startPackage(0, wait_timeout);
      if(ACK::getError(ack))
      {
        DERROR("Failed to start subscription package!\n");
      }
    }
    else
    {
      DERROR("Failed to initialize subscription package!\n");
      return false;
    }

    // Wait for telemetry data
#ifdef QT
    QThread::msleep(200);
#elif STM32
    STM32F4::delay_nms(500);
#else
    sleep(2);
#endif
    subscriptionGimbal =
	  this->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();

    this->subscribe->removePackage(0, wait_timeout);
#ifdef QT
    QThread::msleep(100);
#elif STM32
    STM32F4::delay_nms(500);
#else
    sleep(2);
#endif
  }

  if((this->getFwVersion() != Version::M100_31 &&
      subscriptionGimbal.mountStatus == GIMBAL_MOUNTED) ||
      this->getFwVersion() == Version::M100_31)
  {
    this->gimbal = new (std::nothrow) Gimbal(this);
    if (this->gimbal == 0)
    {
      DERROR("Failed to allocate memory for Gimbal!\n");
      return false;
    }
    return true;
  }
  else
  {
    DERROR("Gimbal not mounted!\n");
  }

  return false;
}

bool
Vehicle::initMFIO()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::mfio))
  {
    mfio = new (std::nothrow) MFIO(this);
    if (this->mfio == 0)
    {
      DERROR("Failed to allocate memory for MFIO!\n");
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool
Vehicle::initMOC()
{
  moc = new (std::nothrow) MobileCommunication(this);
  if (this->moc == 0)
  {
    DERROR("Failed to allocate memory for MobileCommunication!\n");
    return false;
  }

  return true;
}

bool
Vehicle::initMissionManager()
{
  this->missionManager = new (std::nothrow) MissionManager(this);
  if (this->missionManager == 0)
    return false;

  return true;
}

bool
Vehicle::initHardSync()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::hardwareSync))
  {
    hardSync = new (std::nothrow) HardwareSync(this);
    if (this->hardSync == 0)
    {
      return false;
    }

    return true;
  }

  return false;
}

bool
Vehicle::initVirtualRC()
{
  if (isCmdSetSupported(OpenProtocol::CMDSet::virtualRC))
  {
    virtualRC = new (std::nothrow) VirtualRC(this);
    if (this->virtualRC == 0)
    {
      DERROR("Error creating Virtual RC!");
      return false;
    }

    return true;
  }

  return false;
}

bool
Vehicle::isCmdSetSupported(const uint8_t cmdSet)
{
  for (int i = 0; i < sizeof(cmd_setSupportMatrix); i++)
  {
    if (cmd_setSupportMatrix[i].cmdSet == cmdSet)
    {
      if (cmdSet == OpenProtocol::CMDSet::virtualRC &&
          versionData.fwVersion != Version::M100_31)
      {
        return false;
      }
      else if (versionData.fwVersion == Version::M100_31)
      {
        // CMDs not supported in Matrice 100
        if (cmdSet == OpenProtocol::CMDSet::hardwareSync ||
            cmdSet == OpenProtocol::CMDSet::mfio ||
            cmdSet == OpenProtocol::CMDSet::subscribe)
        {
          return false;
        }
      }
    }
  }
  return true;
}

void
Vehicle::processReceivedData(RecvContainer receivedFrame)
{
  receivedFrame.recvInfo.version = this->getFwVersion();
  if (receivedFrame.dispatchInfo.isAck)
  {
    // TODO Fill up ACKErorCode Container
    if (receivedFrame.dispatchInfo.isCallback)
    {
      this->nbVehicleCallBackHandler.callback =
        (VehicleCallBack) this
          ->nbCallbackFunctions[receivedFrame.dispatchInfo.callbackID];
      this->nbVehicleCallBackHandler.userData =
        this->nbUserData[receivedFrame.dispatchInfo.callbackID];

      if (threadSupported)
      {
        this->nbCallbackRecvContainer[receivedFrame.dispatchInfo.callbackID] =
            receivedFrame;
        protocolLayer->getThreadHandle()->lockNonBlockCBAck();
        this->circularBuffer->cbPush(
          this->circularBuffer, this->nbVehicleCallBackHandler,
          this->nbCallbackRecvContainer[receivedFrame.dispatchInfo.callbackID]);
        protocolLayer->getThreadHandle()->freeNonBlockCBAck();
      }
      else
        this->nbVehicleCallBackHandler.callback(
          this,
          receivedFrame,
          this->nbVehicleCallBackHandler.userData);
    }

    else
    {
      DDEBUG("Dispatcher identified as blocking call\n");
      // TODO remove
      this->lastReceivedFrame = receivedFrame;

      ACKHandler(static_cast<void*>(&receivedFrame));
      protocolLayer->getThreadHandle()->notify();
    }
  }
  else
  {
    DDEBUG("Dispatcher identified as push data\n");
    PushDataHandler(static_cast<void*>(&receivedFrame));
  }
}

int
Vehicle::callbackIdIndex()
{
  if (callbackId == 199)
  {
    callbackId = 0;
    return 0;
  }
  else
  {
    callbackId++;
    return callbackId;
  }
}

void
Vehicle::activate(ActivateData* data, VehicleCallBack callback,
                  UserData userData)
{
  data->version        = this->versionData.fwVersion;
  accountData          = *data;
  accountData.reserved = 2;

  for (int i = 0; i < 32; ++i)
  {
    accountData.iosID[i] = '0'; //! @note for ios verification
  }
  DSTATUS("version 0x%X\n", versionData.fwVersion);
  DDEBUG("%.32s", accountData.iosID);
  //! Using function prototype II of send
  int cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)activateCallback;
    nbUserData[cbIndex]          = NULL;
  }
  protocolLayer->send(
    2, 0, OpenProtocol::CMDSet::Activation::activate, (uint8_t*)&accountData,
    sizeof(accountData) - sizeof(char*), 1000, 3, true, cbIndex);
}

ACK::ErrorCode
Vehicle::activate(ActivateData* data, int timeout)
{
  ACK::ErrorCode* ack;
  data->version        = versionData.fwVersion;
  accountData          = *data;
  accountData.reserved = 2;

  for (int i             = 0; i < 32; ++i)
    accountData.iosID[i] = '0'; //! @note for ios verification
  DSTATUS("version 0x%X\n", versionData.fwVersion);
  DDEBUG("%.32s", accountData.iosID);
  //! Using function prototype II of send
  protocolLayer->send(2, 0, OpenProtocol::CMDSet::Activation::activate,
                      (uint8_t*)&accountData,
                      sizeof(accountData) - sizeof(char*), 1000, 3, false, 0);

  ack = (ACK::ErrorCode*)waitForACK(OpenProtocol::CMDSet::Activation::activate,
                                    timeout);

  if (ack->data == OpenProtocol::ErrorCode::ActivationACK::SUCCESS &&
      accountData.encKey)
  {
    DSTATUS("Activation successful\n");
    protocolLayer->setKey(accountData.encKey);
  }
  else
  {
    //! Let user know about other errors if any
    ACK::getErrorCodeMessage(*ack, __func__);
    DERROR("Failed to activate please retry SET 0x%X ID 0x%X code 0x%X\n",
           ack->info.cmd_set, ack->info.cmd_id, ack->data);
  }

  return *ack;
}

void
Vehicle::getDroneVersion(VehicleCallBack callback, UserData userData)
{
  versionData.version_ack =
    OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  versionData.version_crc     = 0x0;
  versionData.version_name[0] = 0;
  versionData.fwVersion = 0;

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 3;
  uint8_t  cmd_data    = 0;
  int      cbIndex     = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)getDroneVersionCallback;
    nbUserData[cbIndex]          = NULL;
  }

  // When UserData is implemented, pass the Vehicle as userData.
  protocolLayer->send(2, 0, OpenProtocol::CMDSet::Activation::getVersion,
                      (uint8_t*)&cmd_data, 1, cmd_timeout, retry_time, true,
                      cbIndex);
}

ACK::DroneVersion
Vehicle::getDroneVersion(int timeout)
{
  versionData.version_ack =
    OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  versionData.version_crc     = 0x0;
  versionData.version_name[0] = 0;

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 3;
  uint8_t  cmd_data    = 0;

  protocolLayer->send(2, 0, OpenProtocol::CMDSet::Activation::getVersion,
                      (uint8_t*)&cmd_data, 1, cmd_timeout, retry_time, false,
                      0);

  // Wait for drone version data
  uint8_t* rawACK =
    (uint8_t*)waitForACK(OpenProtocol::CMDSet::Activation::getVersion, timeout);

  // Parse received data
  if (!parseDroneVersionInfo(this->versionData, rawACK))
  {
    DERROR("Drone version not obtained! Please do not proceed. \n"
           "Check if your serial connection is okay, and if your activation "
           "status is okay.\n");
    //! Set fwVersion to 0 so we can catch the error.
    this->versionData.fwVersion = 0;
  }
  else
  {
    //! Construct final ACK to return to user
    droneVersionACK.ack.data         = this->versionData.version_ack;
    droneVersionACK.data.version_ack = this->versionData.version_ack;
    droneVersionACK.data.version_crc = this->versionData.version_crc;
    droneVersionACK.data.fwVersion   = this->versionData.fwVersion;

    strncpy(droneVersionACK.data.version_name, this->versionData.version_name,
            sizeof(this->versionData.version_name));
    droneVersionACK.data.version_name[sizeof(this->versionData.version_name)] =
      '\0';

    strncpy(droneVersionACK.data.hwVersion, this->versionData.hwVersion,
            sizeof(this->versionData.hwVersion));
    droneVersionACK.data.hwVersion[sizeof(this->versionData.hwVersion)] = '\0';

    strncpy(droneVersionACK.data.hw_serial_num, this->versionData.hw_serial_num,
            sizeof(this->versionData.hw_serial_num));
    droneVersionACK.data.hw_serial_num[sizeof(this->versionData.hwVersion)] =
      '\0';
  }
  return droneVersionACK;
}

Vehicle::ActivateData
Vehicle::getAccountData() const
{
  return accountData;
}

void
Vehicle::setAccountData(const ActivateData& value)
{
  accountData = value;
}

void
Vehicle::activateCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                          UserData userData)
{

  uint16_t ack_data;
  if (recvFrame.recvInfo.len - Protocol::PackageMin <= 2)
  {
    ack_data = recvFrame.recvData.ack;

    vehiclePtr->ackErrorCode.data = ack_data;
    vehiclePtr->ackErrorCode.info = recvFrame.recvInfo;

    if (ACK::getError(vehiclePtr->ackErrorCode) &&
        ack_data == OpenProtocol::ErrorCode::ActivationACK::OSDK_VERSION_ERROR)
    {
      DERROR("SDK version did not match\n");
      vehiclePtr->getDroneVersion();
    }

    //! Let user know about other errors if any
    ACK::getErrorCodeMessage(vehiclePtr->ackErrorCode, __func__);
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }

  if (ack_data == OpenProtocol::ErrorCode::ActivationACK::SUCCESS &&
      vehiclePtr->accountData.encKey)
  {
    vehiclePtr->protocolLayer->setKey(vehiclePtr->accountData.encKey);
  }
}

void
Vehicle::getDroneVersionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                 UserData userData)
{

  if (!parseDroneVersionInfo(vehiclePtr->versionData,
                             recvFrame.recvData.versionACK))
  {
    DERROR("Drone version not obtained! Please do not proceed. \n"
           "Check if your serial connection is okay, and if your activation "
           "status is okay.\n");
    //! Set fwVersion to 0 so we can catch the error.
    vehiclePtr->versionData.fwVersion = 0;
  }
  else
  {
    //! Finally, we print stuff out.
    if (vehiclePtr->versionData.fwVersion > Version::FW(3, 1, 0, 0))
    {
      DSTATUS("Device Serial No. = %.16s\n",
              vehiclePtr->versionData.hw_serial_num);
    }
    DSTATUS("Hardware = %.12s\n", vehiclePtr->versionData.hwVersion);
    DSTATUS("Firmware = %X\n", vehiclePtr->versionData.fwVersion);
    if (vehiclePtr->versionData.fwVersion < Version::FW(3, 2, 0, 0))
    {
      DSTATUS("Version CRC = 0x%X\n", vehiclePtr->versionData.version_crc);
    }
  }
}

void
Vehicle::controlAuthorityCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                  UserData userData)
{
  ACK::ErrorCode ack;
  ack.data = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  uint8_t data    = 0x1;
  int     cbIndex = vehiclePtr->callbackIdIndex();

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                    OBTAIN_CONTROL_IN_PROGRESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    vehiclePtr->obtainCtrlAuthority(controlAuthorityCallback);
  }
  else if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                         RELEASE_CONTROL_IN_PROGRESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    vehiclePtr->releaseCtrlAuthority(controlAuthorityCallback);
  }
  else
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
}

/*****************************Set State
 * Data**************************************/

void
Vehicle::setVersion(const Version::FirmWare& value)
{
  versionData.fwVersion = value;
}

void
Vehicle::setStopCond(bool stopCond)
{
  protocolLayer->getThreadHandle()->lockStopCond();
  this->stopCond = stopCond;
  protocolLayer->getThreadHandle()->freeStopCond();
}

bool
Vehicle::getStopCond()
{
  bool currentStopCondVal;
  protocolLayer->getThreadHandle()->lockStopCond();
  currentStopCondVal = stopCond;
  protocolLayer->getThreadHandle()->freeStopCond();
  return currentStopCondVal;
}

void
Vehicle::ACKHandler(void* eventData)
{
  if (!eventData)
  {
    DERROR("Invalid ACK event data received!\n");
    return;
  }

  RecvContainer* ackData = (RecvContainer*)eventData;
  const uint8_t cmd[] = { ackData->recvInfo.cmd_set, ackData->recvInfo.cmd_id };

  if (ackData->recvInfo.cmd_set == OpenProtocol::CMDSet::mission)
  {
    if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointAddPoint,
               sizeof(cmd)) == 0)
    {
      waypointAddPointACK.ack.info = ackData->recvInfo;
      waypointAddPointACK.ack.data = ackData->recvData.wpAddPointACK.ack;
      waypointAddPointACK.index    = ackData->recvData.wpAddPointACK.index;
    }
    if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointDownload,
               sizeof(cmd)) == 0)
    {
      waypointInitACK.ack.info = ackData->recvInfo;
      waypointInitACK.ack.data = ackData->recvData.wpInitACK.ack;
      waypointInitACK.data     = ackData->recvData.wpInitACK.data;
    }
    if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointIndexDownload,
               sizeof(cmd)) == 0)
    {
      waypointIndexACK.ack.info = ackData->recvInfo;
      waypointIndexACK.ack.data = ackData->recvData.wpIndexACK.ack;
      waypointIndexACK.data     = ackData->recvData.wpIndexACK.data;
    }
    else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::hotpointStart,
                    sizeof(cmd)) == 0)
    {
      hotpointStartACK.ack.info  = ackData->recvInfo;
      hotpointStartACK.ack.data  = ackData->recvData.hpStartACK.ack;
      hotpointStartACK.maxRadius = ackData->recvData.hpStartACK.maxRadius;
    }
    else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::hotpointDownload,
		    sizeof(cmd)) == 0)
    {
      hotpointReadACK.ack.info  = ackData->recvInfo;
      hotpointReadACK.ack.data  = ackData->recvData.hpReadACK.ack;
      hotpointReadACK.data = ackData->recvData.hpReadACK.data;
    }
    else
    {
      ackErrorCode.info = ackData->recvInfo;
      ackErrorCode.data = ackData->recvData.missionACK;
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Activation::getVersion,
                  sizeof(cmd)) == 0)
  {
    size_t arrLength = sizeof(ackData->recvData.versionACK);
    for (int i = 0; i < arrLength; i++)
    {
      //! Interim stage: version data will be parsed before returned to user
      this->rawVersionACK[i] = ackData->recvData.versionACK[i];
    }
    droneVersionACK.ack.info = ackData->recvInfo;
  }
  else if (ackData->recvInfo.cmd_set == OpenProtocol::CMDSet::subscribe)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.subscribeACK;
  }
  else if (ackData->recvInfo.cmd_set == OpenProtocol::CMDSet::control)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.commandACK;
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::MFIO::init, sizeof(cmd)) == 0)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.mfioACK;
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::MFIO::get, sizeof(cmd)) == 0)
  {
    mfioGetACK.ack.info = ackData->recvInfo;
    mfioGetACK.ack.data = ackData->recvData.mfioGetACK.result;
    mfioGetACK.value    = ackData->recvData.mfioGetACK.value;
  }
  else
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.ack;
  }
}

void
Vehicle::PushDataHandler(void* eventData)
{
  RecvContainer* pushDataEntry = (RecvContainer*)eventData;

  const uint8_t cmd[] = { pushDataEntry->recvInfo.cmd_set,
                          pushDataEntry->recvInfo.cmd_id };

  if (memcmp(cmd, OpenProtocol::CMDSet::Broadcast::broadcast, sizeof(cmd)) == 0)
  {
    if (broadcast)
    {
      if (broadcast->unpackHandler.callback)
      {
        broadcast->unpackHandler.callback(this, *(pushDataEntry),
                                          broadcast->unpackHandler.userData);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Broadcast::subscribe,
                  sizeof(cmd)) == 0)
  {
    if (subscribe)
    {
      DDEBUG("Decode callback subscribe");
      if (subscribe->subscriptionDataDecodeHandler.callback)
      {
        subscribe->subscriptionDataDecodeHandler.callback(
          this, *(pushDataEntry),
          subscribe->subscriptionDataDecodeHandler.userData);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Broadcast::fromMobile,
                  sizeof(cmd)) == 0)
  {
    if (moc)
    {
      DDEBUG("Received data from mobile\n");
      if (moc->fromMSDKHandler.callback)
      {
        moc->fromMSDKHandler.callback(this, *(pushDataEntry),
                                      moc->fromMSDKHandler.userData);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Broadcast::mission, sizeof(cmd)) ==
           0)
  {
    if (missionManager)
    {
      if (missionCallback.callback)
      {
        missionCallback.callback(this, *(pushDataEntry),
                                 missionCallback.userData);
      }
      else
      {
        switch (pushDataEntry->recvData.missionACK)
        {
          case MISSION_MODE_A:
            break;
          case MISSION_WAYPOINT:
            if (missionManager->wpMission)
            {
              if (wayPointData)
              {
                if (missionManager->wpMission->wayPointCallback.callback)
                  missionManager->wpMission->wayPointCallback.callback(
                    this, *(pushDataEntry),
                    missionManager->wpMission->wayPointCallback.userData);
                else
                  DDEBUG("Mode WayPoint\n");
              }
            }
            break;
          case MISSION_HOTPOINT:
            if (missionManager->hpMission)
            {
              if (hotPointData)
              {
                if (missionManager->hpMission->hotPointCallback.callback)
                  missionManager->hpMission->hotPointCallback.callback(
                    this, *(pushDataEntry),
                    missionManager->hpMission->hotPointCallback.userData);
                else
                  DDEBUG("Mode HotPoint\n");
              }
            }
            break;
          case MISSION_IOC:
            //! @todo compare IOC with other mission modes comprehensively
            DDEBUG("Mode IOC \n");
            break;
          default:
            DERROR("Unknown mission code 0x%X \n", pushDataEntry->recvData.ack);
            break;
        }
      }
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Broadcast::waypoint,
                  sizeof(cmd)) == 0)
  {
    if (missionManager->wpMission)
    {
      //! @todo add waypoint session decode
      if (missionManager->wpMission->wayPointEventCallback.callback)
      {
        missionManager->wpMission->wayPointEventCallback.callback(
          this, *(pushDataEntry),
          missionManager->wpMission->wayPointEventCallback.userData);
      }
      else
      {
        DDEBUG("WayPoint DATA");
      }
    }
  }
  else
  {
    DDEBUG("Received Unknown PushData\n");
  }
}

void*
Vehicle::waitForACK(const uint8_t (&cmd)[OpenProtocol::MAX_CMD_ARRAY_SIZE],
                    int timeout)
{
  void* pACK;

  protocolLayer->getThreadHandle()->lockACK();
  protocolLayer->getThreadHandle()->wait(timeout);

  if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointAddPoint,
             sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointAddPointACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointDownload,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointInitACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::waypointIndexDownload,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointIndexACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::hotpointStart,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->hotpointStartACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Mission::hotpointDownload,
		  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->hotpointReadACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Activation::getVersion,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->rawVersionACK);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::MFIO::get, sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->mfioGetACK);
  }
  else
  {
    pACK = static_cast<void*>(&this->ackErrorCode);
  }

  protocolLayer->getThreadHandle()->freeACK();

  return pACK;
}

void
Vehicle::obtainCtrlAuthority(VehicleCallBack callback, UserData userData)
{
  uint8_t data    = 1;
  int     cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)controlAuthorityCallback;
    nbUserData[cbIndex]          = NULL;
  }
  protocolLayer->send(2, DJI::OSDK::encrypt,
                      OpenProtocol::CMDSet::Control::setControl, &data, 1, 500,
                      2, true, cbIndex);
}

ACK::ErrorCode
Vehicle::obtainCtrlAuthority(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t         data = 1;

  protocolLayer->send(2, DJI::OSDK::encrypt,
                      OpenProtocol::CMDSet::Control::setControl, &data, 1, 500,
                      2, false, 0);

  ack = *(ACK::ErrorCode*)waitForACK(OpenProtocol::CMDSet::Control::setControl,
                                    timeout);

  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                     OBTAIN_CONTROL_IN_PROGRESS)
  {
    ack = this->obtainCtrlAuthority(timeout);
  }

  return ack;
}

void
Vehicle::releaseCtrlAuthority(VehicleCallBack callback, UserData userData)
{
  uint8_t data    = 0;
  int     cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    // nbCallbackFunctions[cbIndex] = (void*)ReleaseCtrlCallback;
    nbUserData[cbIndex] = NULL;
  }
  protocolLayer->send(2, DJI::OSDK::encrypt,
                      OpenProtocol::CMDSet::Control::setControl, &data, 1, 500,
                      2, true, cbIndex);
}

ACK::ErrorCode
Vehicle::releaseCtrlAuthority(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t         data = 0;

  protocolLayer->send(2, DJI::OSDK::encrypt,
                      OpenProtocol::CMDSet::Control::setControl, &data, 1, 500,
                      2, false, 1);

  ack = *(ACK::ErrorCode*)waitForACK(OpenProtocol::CMDSet::Control::setControl,
                                    timeout);

  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                     RELEASE_CONTROL_IN_PROGRESS)
  {
    ack = this->releaseCtrlAuthority(timeout);
  }

  return ack;
}

void
Vehicle::setLastReceivedFrame(RecvContainer recvFrame)
{
  protocolLayer->getThreadHandle()->lockFrame();
  this->lastReceivedFrame = recvFrame;
  protocolLayer->getThreadHandle()->freeFrame();
}

RecvContainer
Vehicle::getLastReceivedFrame()
{
  RecvContainer recvFrame;
  protocolLayer->getThreadHandle()->lockFrame();
  recvFrame = this->lastReceivedFrame;
  protocolLayer->getThreadHandle()->freeFrame();
  return recvFrame;
}

Version::FirmWare
Vehicle::getFwVersion() const
{
  return versionData.fwVersion;
}
char*
Vehicle::getHwVersion() const
{
  return (char*)versionData.hwVersion;
}
char*
Vehicle::getHwSerialNum() const
{
  return (char*)versionData.hw_serial_num;
}
