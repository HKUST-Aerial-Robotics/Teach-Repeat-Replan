/** @file dji_open_protocol.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  OPEN Protocol implementation for DJI Onboard SDK library
 *
 *  @copyright 2017 DJI. All right reserved.
 *
 */
#include "dji_open_protocol.hpp"

#ifdef STM32
#include <stdio.h>
#endif

using namespace DJI;
using namespace DJI::OSDK;

//! Constructor
Protocol::Protocol(const char* device, uint32_t baudrate)
{
//! Step 1: Initialize Hardware Driver

//! Step 1.1: Instantiate a hardware driver as per OS
#ifdef QT
  QThread* serialEventThread = new QThread;
  QHardDriver* driver = new QHardDriver(0, device, baudrate);
  driver->moveToThread(serialEventThread);
  QObject::connect(serialEventThread, SIGNAL(started()), driver, SLOT(init()));
  QObject::connect(driver, SIGNAL (finished()), driver, SLOT (deleteLater()));

  QObject::connect(serialEventThread, SIGNAL (finished()), serialEventThread, SLOT (deleteLater()));
  serialEventThread->start();
  QThread::msleep(100);
  this->serialDevice = driver;
  this->threadHandle = new QThreadManager();
//! Add correct Qt serial device constructor here
#elif STM32
  this->serialDevice = new STM32F4;
  this->threadHandle = new STM32F4DataGuard;
#elif defined(__linux__)
  this->serialDevice = new LinuxSerialDevice(device, baudrate);
  this->threadHandle = new PosixThreadManager();
#endif

  //! Step 1.2: Initialize the hardware driver
#ifndef QT
  this->serialDevice->init();
#endif
  this->threadHandle->init();

  //! Step 2: Initialize the ProtocolLayer
  init(this->serialDevice, this->serialDevice->getMmu());
}

/***************************Init*******************************************/
void
Protocol::init(HardDriver* sDevice, MMU* mmuPtr, bool userCallbackThread)
{

  serialDevice = sDevice;

  seq_num              = 0;
  ackFrameStatus       = 11;
  broadcastFrameStatus = false;

  filter.recvIndex  = 0;
  filter.reuseCount = 0;
  filter.reuseIndex = 0;
  filter.encode     = 0;

  /* Still up for discussion: Is this mechanism useful?
  recvCallback.callback = userRecvCallback.callback;
  recvCallback.userData = userRecvCallback.userData;
  */

  /* Callback thread: come back to this later. We probably need to move this to
  a higher layer.

  callbackThread = false;
  callbackThread = userCallbackThread;  //! @todo implement

  nonBlockingCBThreadEnable = false;
  */

  mmu          = mmuPtr;
  buf_read_pos = 0;
  read_len     = 0;

  setup();
}

/*********************************Session &
 * memory******************************/

void
Protocol::setup()
{
  mmu->setupMMU();
  setupSession();
}

void
Protocol::setupSession()
{
  uint32_t i;
  for (i = 0; i < SESSION_TABLE_NUM; i++)
  {
    CMDSessionTab[i].sessionID = i;
    CMDSessionTab[i].usageFlag = 0;
    CMDSessionTab[i].mmu       = (MMU_Tab*)NULL;
  }

  for (i = 0; i < (SESSION_TABLE_NUM - 1); i++)
  {
    ACKSessionTab[i].sessionID     = i + 1;
    ACKSessionTab[i].sessionStatus = ACK_SESSION_IDLE;
    ACKSessionTab[i].mmu           = (MMU_Tab*)NULL;
  }
}

CMDSession*
Protocol::allocSession(uint16_t session_id, uint16_t size)
{
  uint32_t i;
  DDEBUG("Allocation size %d", size);
  MMU_Tab* memoryTab = NULL;

  if (session_id == 0 || session_id == 1)
  {
    if (this->CMDSessionTab[session_id].usageFlag == 0)
      i = session_id;
    else
    {
      /* session is busy */
      DERROR("session %d is busy\n", session_id);
      return NULL;
    }
  }
  else
  {
    for (i = 2; i < SESSION_TABLE_NUM; i++)
      if (CMDSessionTab[i].usageFlag == 0)
        break;
  }
  if (i < 32 && CMDSessionTab[i].usageFlag == 0)
  {
    CMDSessionTab[i].usageFlag = 1;
    memoryTab                  = mmu->allocMemory(size);
    if (memoryTab == NULL)
      CMDSessionTab[i].usageFlag = 0;
    else
    {
      CMDSessionTab[i].mmu = memoryTab;
      return &CMDSessionTab[i];
    }
  }
  return NULL;
}

void
Protocol::freeSession(CMDSession* session)
{
  if (session->usageFlag == 1)
  {
    DDEBUG("session id %d\n", session->sessionID);
    mmu->freeMemory(session->mmu);
    session->usageFlag = 0;
  }
}

ACKSession*
Protocol::allocACK(uint16_t session_id, uint16_t size)
{
  MMU_Tab* memoryTab = NULL;
  if (session_id > 0 && session_id < 32)
  {
    if (ACKSessionTab[session_id - 1].mmu)
      freeACK(&ACKSessionTab[session_id - 1]);
    memoryTab = mmu->allocMemory(size);
    if (memoryTab == NULL)
    {
      DERROR("there is not enough memory\n");
      return NULL;
    }
    else
    {
      ACKSessionTab[session_id - 1].mmu = memoryTab;
      return &ACKSessionTab[session_id - 1];
    }
  }
  DERROR("wrong Ack session ID: 0x%X\n", session_id);
  return NULL;
}

void
Protocol::freeACK(ACKSession* session)
{
  mmu->freeMemory(session->mmu);
}

/*********************************Send
 * Pipeline**********************************/

//! v1: no handler? This definition seems weird.
/*
void Protocol::send(uint8_t session, uint8_t is_enc,
DJI_CONTROLLER_CMD cmdSet,
                         uint8_t cmdID, void *pdata, int len,
                         bool isCallback, int callbackID, int timeout, int
retry) {
  Command cmdContainer;
  uint8_t *ptemp = (uint8_t *)encodeSendData;
  *ptemp++             = cmdSet;
  *ptemp++             = cmdID;

  memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

  //@todo Replace with a bool
  //cmdContainer.handler     = ackCallback;
  cmdContainer.sessionMode = session;
  cmdContainer.length      = len + SET_CMD_SIZE;
  cmdContainer.buf         = encodeSendData;
  cmdContainer.retry       = retry;

  cmdContainer.timeout = timeout;
  cmdContainer.encrypt = is_enc;

  //@todo This will go away too.
  cmdContainer.userData = 0;

  sendInterface(&cmdContainer);
}
*/
//! v2 : This is more complete
void
Protocol::send(uint8_t session_mode, bool is_enc, const uint8_t cmd[],
               void* pdata, size_t len, int timeout, int retry_time,
               bool hasCallback, int callbackID)
{
  Command  cmdContainer;
  uint8_t* ptemp = (uint8_t*)encodeSendData;
  *ptemp++       = cmd[0];
  *ptemp++       = cmd[1];

  memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

  cmdContainer.sessionMode = session_mode;
  cmdContainer.length      = len + SET_CMD_SIZE;
  cmdContainer.buf         = encodeSendData;
  cmdContainer.cmd_set     = cmd[0]; // cmd set
  cmdContainer.cmd_id      = cmd[1]; // cmd id
  cmdContainer.retry       = retry_time;

  cmdContainer.timeout = timeout;
  cmdContainer.encrypt = is_enc ? 1 : 0;

  //! Callback
  cmdContainer.isCallback = hasCallback;
  cmdContainer.callbackID = callbackID;

  sendInterface(&cmdContainer);
}

//! v3: Minimal
void
Protocol::send(Command* cmdContainer)
{
  sendInterface(cmdContainer);
}

int
Protocol::sendInterface(Command* cmdContainer)
{
  uint16_t    ret        = 0;
  CMDSession* cmdSession = (CMDSession*)NULL;
  if (cmdContainer->length > PRO_PURE_DATA_MAX_SIZE)
  {
    DERROR("ERROR,length=%lu is over-sized\n", cmdContainer->length);
    return -1;
  }
  /*! Switch on session to decide whether the command is requesting an ACK and
   * whether it is requesting
   *  guarantees on transmission
   */

  switch (cmdContainer->sessionMode)
  {
    case 0:
      //! No ACK required and no retries
      threadHandle->lockMemory();
      cmdSession =
        allocSession(CMD_SESSION_0, calculateLength(cmdContainer->length,
                                                    cmdContainer->encrypt));

      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeMemory();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      //! Encrypt the data being sent
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);
      if (ret == 0)
      {
        DERROR("encrypt ERROR\n");
        freeSession(cmdSession);
        threadHandle->freeMemory();
        return -1;
      }

      DDEBUG("send data in session mode 0\n");

      //! Actually send the data
      sendData(cmdSession->mmu->pmem);
      seq_num++;
      freeSession(cmdSession);
      threadHandle->freeMemory();
      break;

    case 1:
      //! ACK required; Session 1; will retry until failure
      threadHandle->lockMemory();
      cmdSession =
        allocSession(CMD_SESSION_1, calculateLength(cmdContainer->length,
                                                    cmdContainer->encrypt));
      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeMemory();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      if (seq_num == cmdSession->preSeqNum)
      {
        seq_num++;
      }
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);
      if (ret == 0)
      {
        DERROR("encrypt ERROR\n");
        freeSession(cmdSession);
        threadHandle->freeMemory();
        return -1;
      }
      cmdSession->preSeqNum = seq_num++;

      //@todo replace with a bool
      cmdSession->isCallback = cmdContainer->isCallback;
      cmdSession->callbackID = cmdContainer->callbackID;
      cmdSession->timeout =
        (cmdContainer->timeout > POLL_TICK) ? cmdContainer->timeout : POLL_TICK;
      cmdSession->preTimestamp = serialDevice->getTimeStamp();
      cmdSession->sent         = 1;
      cmdSession->retry        = 1;
      DDEBUG("sending session %d\n", cmdSession->sessionID);
      sendData(cmdSession->mmu->pmem);
      threadHandle->freeMemory();
      break;

    case 2:
      //! ACK required, Sessions 2 - END; no guarantees and no retries.
      threadHandle->lockMemory();
      cmdSession =
        allocSession(CMD_SESSION_AUTO, calculateLength(cmdContainer->length,
                                                       cmdContainer->encrypt));
      if (cmdSession == (CMDSession*)NULL)
      {
        threadHandle->freeMemory();
        DERROR("ERROR,there is not enough memory\n");
        return -1;
      }
      if (seq_num == cmdSession->preSeqNum)
      {
        seq_num++;
      }
      ret =
        encrypt(cmdSession->mmu->pmem, cmdContainer->buf, cmdContainer->length,
                0, cmdContainer->encrypt, cmdSession->sessionID, seq_num);

      if (ret == 0)
      {
        DERROR("encrypt ERROR");
        freeSession(cmdSession);
        threadHandle->freeMemory();
        return -1;
      }

      // To use in ErrorCode manager
      cmdSession->cmd_set = cmdContainer->cmd_set;
      cmdSession->cmd_id  = cmdContainer->cmd_id;
      // Will carry information: obtain/release control
      cmdSession->buf = cmdContainer->buf;

      cmdSession->preSeqNum = seq_num++;
      //@todo replace with a bool
      cmdSession->isCallback = cmdContainer->isCallback;
      cmdSession->callbackID = cmdContainer->callbackID;
      cmdSession->timeout =
        (cmdContainer->timeout > POLL_TICK) ? cmdContainer->timeout : POLL_TICK;
      cmdSession->preTimestamp = serialDevice->getTimeStamp();
      cmdSession->sent         = 1;
      cmdSession->retry        = cmdContainer->retry;
      DDEBUG("Sending session %d\n", cmdSession->sessionID);
      sendData(cmdSession->mmu->pmem);
      threadHandle->freeMemory();
      break;
    default:
      DERROR("Unknown mode:%d\n", cmdContainer->sessionMode);
      break;
  }
  return 0;
}

void
Protocol::sendData(uint8_t* buf)
{
  size_t  ans;
  Header* pHeader = (Header*)buf;

#ifdef API_TRACE_DATA
  printFrame(serialDevice, pHeader, true);
#endif

  //! Serial Device call: last link in the send pipeline
  ans = serialDevice->send(buf, pHeader->length);
  if (ans == 0)
    DSTATUS("Port did not send");
  if (ans == (size_t)-1)
    DERROR("Port closed");
}

//! Session management for the send pipeline: Poll

void
Protocol::sendPoll()
{
  uint8_t i;
  time_ms curTimestamp;
  for (i = 1; i < SESSION_TABLE_NUM; i++)
  {
    if (CMDSessionTab[i].usageFlag == 1)
    {
      curTimestamp = serialDevice->getTimeStamp();
      if ((curTimestamp - CMDSessionTab[i].preTimestamp) >
          CMDSessionTab[i].timeout)
      {
        threadHandle->lockMemory();
        if (CMDSessionTab[i].retry > 0)
        {
          if (CMDSessionTab[i].sent >= CMDSessionTab[i].retry)
          {
            DSTATUS("Sending timeout, Free session %d\n",
                    CMDSessionTab[i].sessionID);
            freeSession(&CMDSessionTab[i]);
          }
          else
          {
            DDEBUG("Retry session %d\n", CMDSessionTab[i].sessionID);
            sendData(CMDSessionTab[i].mmu->pmem);
            CMDSessionTab[i].preTimestamp = curTimestamp;
            CMDSessionTab[i].sent++;
          }
        }
        else
        {
          DDEBUG("Send once %d\n", i);
          sendData(CMDSessionTab[i].mmu->pmem);
          CMDSessionTab[i].preTimestamp = curTimestamp;
        }
        threadHandle->freeMemory();
      }
      else
      {
        DDEBUG("Wait for timeout Session: %d \n", i);
      }
    }
  }
  //! @note Add auto resendpoll
}

/*******************************Receive
 * Pipeline*************************************/

//! Step 0: Call this in a loop.
RecvContainer
Protocol::receive()
{
  //! Create a local container that will be used for storing data lower down in
  //! the stack
  RecvContainer receiveFrame;
  receiveFrame.recvInfo.cmd_id = 0xFF;

  //! Run the readPoll until you get a true
  // @todo might need to modify to include thread stopCond
  while (!readPoll(&receiveFrame));
  //! When we receive a true, return a copy of container to the caller: this is
  //! the 'receive' interface

  return receiveFrame;
}

//! Step 1
bool
Protocol::readPoll(RecvContainer* allocatedFramePtr)
{
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  //! Step 1: Check if the buffer has been consumed
  if (buf_read_pos >= read_len)
  {

    this->buf_read_pos = 0;
    this->read_len     = serialDevice->readall(this->buf, BUFFER_SIZE);
  }

#ifdef API_BUFFER_DATA
  onceRead = read_len;
  totalRead += onceRead;
#endif // API_BUFFER_DATA

  //! Step 2: Go through the buffer and return when you see a full frame.
  //! buf_read_pos will maintain state about how much buffer data we have
  //! already read
  for (this->buf_read_pos; this->buf_read_pos < this->read_len;
       this->buf_read_pos++)
  {
    isFrame = byteHandler(buf[this->buf_read_pos], allocatedFramePtr);
    if (isFrame)
    {
      return isFrame;
    }
  }

  //! Step 3: If we don't find a full frame by this time, return false.
  //! The receive function calls readPoll in a loop, so if it returns false
  //! it'll just be called again
  return isFrame;
}

//! Step 2
bool
Protocol::byteHandler(const uint8_t in_data, RecvContainer* allocatedFramePtr)
{
  filter.reuseCount = 0;
  filter.reuseIndex = Protocol::maxRecv;

  RecvContainer* recvDataPtr;
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = streamHandler(&filter, in_data, allocatedFramePtr);

  /*! @note Just think a command as below
    *
    * [123456HHD1234567===HHHH------------------] --- is buf un-used part
    *
    * if after recv full of above, but crc failed, we throw all data?
    * NO!
    * Just throw ONE BYTE, we move like below
    *
    * [123456HH------------------D1234567===HHHH]
    *
    * Use the buffer high part to re-loop, try to find a new command
    *
    * if new cmd also fail, and buf like below
    *
    * [56HHD1234567----------------------===HHHH]
    *
    * throw one byte, buf looks like
    *
    * [6HHD123-----------------------4567===HHHH]
    *
    * the command tail part move to buffer right
    * */
  if (filter.reuseCount != 0)
  {
    while (filter.reuseIndex < Protocol::maxRecv)
    {
      /*! @note because reuse_index maybe re-located, so reuse_index must
       * be
       *  always point to un-used index
       *  re-loop the buffered data
       *  */
      isFrame = streamHandler(&filter, filter.recvBuf[filter.reuseIndex++],
                              allocatedFramePtr);
    }
    filter.reuseCount = 0;
  }
  return isFrame;
}

//! Step 3
bool
Protocol::streamHandler(SDKFilter* p_filter, uint8_t in_data,
                        RecvContainer* allocatedRecvObject)
{
  storeData(p_filter, in_data);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = checkStream(p_filter, allocatedRecvObject);
  return isFrame;
}

//! Step 4
//! @note push data to filter buffer.
//! SDKFilter is just a buffer.
void
Protocol::storeData(SDKFilter* p_filter, uint8_t in_data)
{
  if (p_filter->recvIndex < Protocol::maxRecv)
  {
    p_filter->recvBuf[p_filter->recvIndex] = in_data;
    p_filter->recvIndex++;
  }
  else
  {
    DERROR("buffer overflow");
    memset(p_filter->recvBuf, 0, p_filter->recvIndex);
    p_filter->recvIndex = 0;
  }
}

//! Step 5
bool
Protocol::checkStream(SDKFilter* p_filter, RecvContainer* allocatedRecvObject)
{
  Header* p_head = (Header*)(p_filter->recvBuf);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;
  if (p_filter->recvIndex < sizeof(Header))
  {
    // Continue receive data, nothing to do
    return false;
  }
  else if (p_filter->recvIndex == sizeof(Header))
  {
    // recv a full-head
    isFrame = verifyHead(p_filter, allocatedRecvObject);
  }
  else if (p_filter->recvIndex == p_head->length)
  {
    isFrame = verifyData(p_filter, allocatedRecvObject);
  }
  return isFrame;
}

//! Step 6
bool
Protocol::verifyHead(SDKFilter* p_filter, RecvContainer* allocatedRecvObject)
{
  Header* p_head = (Header*)(p_filter->recvBuf);
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if ((p_head->sof == Protocol::SOF) && (p_head->version == 0) &&
      (p_head->length < Protocol::maxRecv) && (p_head->reserved0 == 0) &&
      (p_head->reserved1 == 0) &&
      (_SDK_CALC_CRC_HEAD(p_head, sizeof(Header)) == 0))
  {
    // check if this head is a ack or simple package
    if (p_head->length == sizeof(Header))
    {
      isFrame = callApp(p_filter, allocatedRecvObject);
    }
  }
  else
  {
    sdk_stream_shift_data_lambda(p_filter);
  }
  return isFrame;
}

//! Step 7
bool
Protocol::verifyData(SDKFilter* p_filter, RecvContainer* allocatedRecvObject)
{
  Header* p_head = (Header*)(p_filter->recvBuf);

  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if (_SDK_CALC_CRC_TAIL(p_head, p_head->length) == 0)
  {
    isFrame = callApp(p_filter, allocatedRecvObject);
  }
  else
  {
    //! @note data crc fail, re-use the data part
    sdk_stream_update_reuse_part_lambda(p_filter);
  }
  return isFrame;
}

//! Step 8
bool
Protocol::callApp(SDKFilter* p_filter, RecvContainer* allocatedRecvObject)
{
  // pass current data to handler
  Header* p_head = (Header*)p_filter->recvBuf;

  encodeData(p_filter, p_head, aes256_decrypt_ecb);
  bool isFrame = appHandler((Header*)p_filter->recvBuf, allocatedRecvObject);
  sdk_stream_prepare_lambda(p_filter);

  return isFrame;
}

//! Step 9
bool
Protocol::appHandler(Header* protocolHeader, RecvContainer* allocatedRecvObject)
{
//! @todo Filter replacement
#ifdef API_TRACE_DATA
  printFrame(serialDevice, protocolHeader, false);
#endif

  Header* p2protocolHeader;
  //! Bool to check if the protocol parser has finished a full frame
  bool isFrame = false;

  if (protocolHeader->isAck == 1)
  {
    //! Case 0: This is an ACK frame that came in.
    if (protocolHeader->sessionID > 1 && protocolHeader->sessionID < 32)
    {
      //! Session is valid
      if (CMDSessionTab[protocolHeader->sessionID].usageFlag == 1)
      {
        //! Session in use
        threadHandle->lockMemory();
        p2protocolHeader =
          (Header*)CMDSessionTab[protocolHeader->sessionID].mmu->pmem;
        if (p2protocolHeader->sessionID == protocolHeader->sessionID &&
            p2protocolHeader->sequenceNumber == protocolHeader->sequenceNumber)
        {
          DDEBUG("Recv Session %d ACK\n", p2protocolHeader->sessionID);

          //! Create receive container for error code management
          allocatedRecvObject->dispatchInfo.isAck = true;
          allocatedRecvObject->recvInfo.cmd_set =
            CMDSessionTab[protocolHeader->sessionID].cmd_set;
          allocatedRecvObject->recvInfo.cmd_id =
            CMDSessionTab[protocolHeader->sessionID].cmd_id;
          allocatedRecvObject->recvData = allocateACK(protocolHeader);
          allocatedRecvObject->dispatchInfo.isCallback =
            CMDSessionTab[protocolHeader->sessionID].isCallback;
          allocatedRecvObject->dispatchInfo.callbackID =
            CMDSessionTab[protocolHeader->sessionID].callbackID;
          allocatedRecvObject->recvInfo.buf =
            CMDSessionTab[protocolHeader->sessionID].buf;
          allocatedRecvObject->recvInfo.seqNumber =
            protocolHeader->sequenceNumber;
          allocatedRecvObject->recvInfo.len = protocolHeader->length;
          //! Set bool
          isFrame = true;

          //! Finish the session
          freeSession(&CMDSessionTab[protocolHeader->sessionID]);
          threadHandle->freeMemory();
          /**
           * Set end of ACK frame
           * @todo Implement proper notification mechanism
           */
          setACKFrameStatus(
            (&CMDSessionTab[protocolHeader->sessionID])->usageFlag);
        }
        else
        {
          threadHandle->freeMemory();
        }
      }
    }
  }
  else
  {
    //! Not an ACK frame
    switch (protocolHeader->sessionID)
    {
      case 0:
        isFrame = recvReqData(protocolHeader, allocatedRecvObject);
        break;
      case 1:
      //! @todo unnecessary ack in case 1. Maybe add code later
      //! @todo check algorithm,
      //! @attention here real have a bug about self-looping issue.
      //! @bug not affect OSDK currerently. 2017-1-18
      default: //! @note session id is 2
        DSTATUS("ACK %d", protocolHeader->sessionID);

        if (ACKSessionTab[protocolHeader->sessionID - 1].sessionStatus ==
            ACK_SESSION_PROCESS)
        {
          DDEBUG("This session is waiting for App ACK:"
                 "session id=%d,seq_num=%d\n",
                 protocolHeader->sessionID, protocolHeader->sequenceNumber);
        }
        else if (ACKSessionTab[protocolHeader->sessionID - 1].sessionStatus ==
                 ACK_SESSION_IDLE)
        {
          if (protocolHeader->sessionID > 1)
            ACKSessionTab[protocolHeader->sessionID - 1].sessionStatus =
              ACK_SESSION_PROCESS;
          isFrame = recvReqData(protocolHeader, allocatedRecvObject);
        }
        else if (ACKSessionTab[protocolHeader->sessionID - 1].sessionStatus ==
                 ACK_SESSION_USING)
        {
          threadHandle->lockMemory();
          p2protocolHeader =
            (Header*)ACKSessionTab[protocolHeader->sessionID - 1].mmu->pmem;
          if (p2protocolHeader->sequenceNumber ==
              protocolHeader->sequenceNumber)
          {
            DDEBUG("Repeat ACK to remote,session "
                   "id=%d,seq_num=%d\n",
                   protocolHeader->sessionID, protocolHeader->sequenceNumber);
            sendData(ACKSessionTab[protocolHeader->sessionID - 1].mmu->pmem);
            threadHandle->freeMemory();
          }
          else
          {
            DDEBUG("Same session,but new seq_num pkg,session id=%d,"
                   "pre seq_num=%d,cur seq_num=%d\n",
                   protocolHeader->sessionID, p2protocolHeader->sequenceNumber,
                   protocolHeader->sequenceNumber);
            ACKSessionTab[protocolHeader->sessionID - 1].sessionStatus =
              ACK_SESSION_PROCESS;
            threadHandle->freeMemory();
            isFrame = recvReqData(protocolHeader, allocatedRecvObject);
          }
        }
        break;
    }
  }
  return isFrame;
}

ACK::TypeUnion
Protocol::allocateACK(Header* protocolHeader)
{

  ACK::TypeUnion recvData;

  if (protocolHeader->length <= MAX_ACK_SIZE)
  {
    memcpy(recvData.raw_ack_array, ((uint8_t*)protocolHeader) + sizeof(Header),
           (protocolHeader->length - Protocol::PackageMin));
  }
  else
  {
    //! @note throw not supported in STM32
    // throw std::runtime_error("Unknown ACK");
  }

  return recvData;
}

void
Protocol::setACKFrameStatus(uint32_t usageFlag)
{
  ackFrameStatus = usageFlag;
}

/********************************Receive CMD
 * Functions***********************************/

uint8_t
Protocol::getCmdSet(Header* protocolHeader)
{
  uint8_t* ptemp = ((uint8_t*)protocolHeader) + sizeof(Header);
  return *ptemp;
}

uint8_t
Protocol::getCmdCode(Header* protocolHeader)
{
  uint8_t* ptemp = ((uint8_t*)protocolHeader) + sizeof(Header);
  ptemp++;
  return *ptemp;
}

//! Step 10: In case we received a CMD frame and not an ACK frame
bool
Protocol::recvReqData(Header*        protocolHeader,
                      RecvContainer* allocatedRecvObject)
{
  uint8_t buf[100] = { 0, 0 };

  //@todo: Please monitor lengths to see whether we need to change the max size
  // of RecvContainer.recvData
  allocatedRecvObject->dispatchInfo.isAck = false;
  uint8_t* payload = (uint8_t*)protocolHeader + sizeof(Header) + 2;
  allocatedRecvObject->recvInfo.cmd_set = getCmdSet(protocolHeader);
  allocatedRecvObject->recvInfo.cmd_id  = getCmdCode(protocolHeader);
  allocatedRecvObject->recvInfo.len     = protocolHeader->length;
  //@todo: Please monitor to make sure the length is correct
  memcpy(allocatedRecvObject->recvData.raw_ack_array, payload,
         ((protocolHeader->length) - (Protocol::PackageMin + 2)));
  allocatedRecvObject->dispatchInfo.isCallback = false;
  allocatedRecvObject->dispatchInfo.callbackID = 0;

  //! isFrame = true
  return true;
}

/*******************************Utility
 * Functions************************************/
uint16_t
Protocol::calculateLength(uint16_t size, uint16_t encrypt_flag)
{
  uint16_t len;
  if (encrypt_flag)
    len = size + sizeof(Header) + 4 + (16 - size % 16);
  else
    len = size + sizeof(Header) + 4;
  return len;
}

void
Protocol::transformTwoByte(const char* pstr, uint8_t* pdata)
{
  int      i;
  char     temp_area[3];
  uint32_t temp8;
  temp_area[0] = temp_area[1] = temp_area[2] = 0;

  for (i = 0; i < 32; i++)
  {
    temp_area[0] = pstr[0];
    temp_area[1] = pstr[1];
    sscanf(temp_area, "%x", &temp8);
    pdata[i] = temp8;
    pstr += 2;
  }
}

/******************************CRC
 * Calculationns*************************************/

void
Protocol::calculateCRC(void* p_data)
{
  Header*  p_head = (Header*)p_data;
  uint8_t* p_byte = (uint8_t*)p_data;
  uint32_t index_of_crc32;

  if (p_head->sof != Protocol::SOF)
    return;
  if (p_head->version != 0)
    return;
  if (p_head->length > Protocol::maxRecv)
    return;
  if (p_head->length > sizeof(Header) && p_head->length < Protocol::PackageMin)
    return;

  p_head->crc = sdk_stream_crc16_calc(p_byte, Protocol::CRCHeadLen);

  if (p_head->length >= Protocol::PackageMin)
  {
    index_of_crc32 = p_head->length - Protocol::CRCData;
    _SDK_U32_SET(p_byte + index_of_crc32,
                 sdk_stream_crc32_calc(p_byte, index_of_crc32));
  }
}

uint16_t
Protocol::crc16_update(uint16_t crc, uint8_t ch)
{
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & static_cast<uint16_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}

uint32_t
Protocol::crc32_update(uint32_t crc, uint8_t ch)
{
  uint32_t tmp;
  uint32_t msg;

  msg = 0x000000ffL & static_cast<uint32_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
  return crc;
}

uint16_t
Protocol::sdk_stream_crc16_calc(const uint8_t* pMsg, size_t nLen)
{
  size_t   i;
  uint16_t wCRC = CRC_INIT;

  for (i = 0; i < nLen; i++)
  {
    wCRC = crc16_update(wCRC, pMsg[i]);
  }

  return wCRC;
}

uint32_t
Protocol::sdk_stream_crc32_calc(const uint8_t* pMsg, size_t nLen)
{
  size_t   i;
  uint32_t wCRC = CRC_INIT;

  for (i = 0; i < nLen; i++)
  {
    wCRC = crc32_update(wCRC, pMsg[i]);
  }

  return wCRC;
}

void
Protocol::sdk_stream_prepare_lambda(SDKFilter* p_filter)
{
  uint32_t bytes_to_move = sizeof(Header) - 1;
  uint32_t index_of_move = p_filter->recvIndex - bytes_to_move;

  memmove(p_filter->recvBuf, p_filter->recvBuf + index_of_move, bytes_to_move);
  memset(p_filter->recvBuf + bytes_to_move, 0, index_of_move);
  p_filter->recvIndex = bytes_to_move;
}

void
Protocol::sdk_stream_shift_data_lambda(SDKFilter* p_filter)
{
  if (p_filter->recvIndex)
  {
    p_filter->recvIndex--;
    if (p_filter->recvIndex)
    {
      memmove(p_filter->recvBuf, p_filter->recvBuf + 1, p_filter->recvIndex);
    }
  }
}

// this function will move the data part to buffer end,
// head part will move left
//
//  1. there no re-use data
//  |------------------------------------------| <= cache
//                       ^
//                       reuse_index
//  [12345678][ data Part ]--------------------| 1. p_filter
//  [12345678]---------------------[ data Part ] 2. move data to end
//  [2345678]----------------------[ data Part ] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
//  2. already has re-use data
//  |---------------------------------[rev data] <= cache
//                  ^
//                  reuse_index, the data already used
//  [12345678][ data Part ]-----------[rev data] 1. p_filter
//  [12345678]-----------[ data Part ][rev data] 2. move data to end
//  [2345678]------------[ data Part ][rev data] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
// the re-use data will loop later

void
Protocol::sdk_stream_update_reuse_part_lambda(SDKFilter* p_filter)
{
  uint8_t* p_buf         = p_filter->recvBuf;
  uint16_t bytes_to_move = p_filter->recvIndex - sizeof(Header);
  uint8_t* p_src         = p_buf + sizeof(Header);

  uint16_t n_dest_index = p_filter->reuseIndex - bytes_to_move;
  uint8_t* p_dest       = p_buf + n_dest_index;

  memmove(p_dest, p_src, bytes_to_move);

  p_filter->recvIndex = sizeof(Header);
  sdk_stream_shift_data_lambda(p_filter);

  p_filter->reuseIndex = n_dest_index;
  p_filter->reuseCount++;
}

/***********************************Encryption****************************************/

void
Protocol::encodeData(SDKFilter* p_filter, Header* p_head,
                     ptr_aes256_codec codec_func)
{
  aes256_context ctx;
  uint32_t       buf_i;
  uint32_t       loop_blk;
  uint32_t       data_len;
  uint32_t       data_idx;
  uint8_t*       data_ptr;

  if (p_head->enc == 0)
    return;
  if (p_head->length <= Protocol::PackageMin)
    return;

  data_ptr = (uint8_t*)p_head + sizeof(Header);
  data_len = p_head->length - Protocol::PackageMin;
  loop_blk = data_len / 16;
  data_idx = 0;

  aes256_init(&ctx, p_filter->sdkKey);
  for (buf_i = 0; buf_i < loop_blk; buf_i++)
  {
    codec_func(&ctx, data_ptr + data_idx);
    data_idx += 16;
  }
  aes256_done(&ctx);

  if (codec_func == aes256_decrypt_ecb)
    p_head->length = p_head->length - p_head->padding; // minus padding length;
}

uint16_t
Protocol::encrypt(uint8_t* pdest, const uint8_t* psrc, uint16_t w_len,
                  uint8_t is_ack, uint8_t is_enc, uint8_t session_id,
                  uint16_t seq_num)
{
  uint16_t data_len;

  Header* p_head = (Header*)pdest;

  if (w_len > 1024)
    return 0;

  if (filter.encode == 0 && is_enc)
  {
    DERROR("Can not send encode data, Please activate your device to get an "
           "available key.\n");
    return 0;
  }
  if (w_len == 0 || psrc == 0)
    data_len = static_cast<uint16_t>(sizeof(Header));
  else
    data_len =
      static_cast<uint16_t>(sizeof(Header) + Protocol::CRCData + w_len);

  if (is_enc)
    data_len = data_len + (16 - w_len % 16);

  DDEBUG("data len: %d\n", data_len);

  p_head->sof       = Protocol::SOF;
  p_head->length    = data_len;
  p_head->version   = 0;
  p_head->sessionID = session_id;
  p_head->isAck     = is_ack ? 1 : 0;
  p_head->reserved0 = 0;

  p_head->padding   = is_enc ? (16 - w_len % 16) : 0;
  p_head->enc       = is_enc ? 1 : 0;
  p_head->reserved1 = 0;

  p_head->sequenceNumber = seq_num;
  p_head->crc            = 0;

  if (psrc && w_len)
    memcpy(pdest + sizeof(Header), psrc, w_len);
  encodeData(&filter, p_head, aes256_encrypt_ecb);

  calculateCRC(pdest);

  return data_len;
}

/*********************************Getters/Setters***********************************/

HardDriver*
Protocol::getDriver() const
{
  return this->serialDevice;
}

ThreadAbstract*
Protocol::getThreadHandle() const
{
  return this->threadHandle;
}

int
Protocol::getBufReadPos()
{
  return buf_read_pos;
}

int
Protocol::getReadLen()
{
  return read_len;
}

/**********************************Filter*******************************************/
void
Protocol::setKey(const char* key)
{
  transformTwoByte(key, filter.sdkKey);
  filter.encode = 1;
}
