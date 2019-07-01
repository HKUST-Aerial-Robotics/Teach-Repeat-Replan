/** @file dji_open_protocol.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  OPEN Protocol implementation for DJI Onboard SDK library
 *
 *  @copyright 2017 DJI. All right reserved.
 *
 */

#ifndef ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H
#define ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H

#include "dji_ack.hpp"
#include "dji_aes.hpp"
#include "dji_hard_driver.hpp"
#include "dji_log.hpp"
#include "dji_thread_manager.hpp"
#include "dji_type.hpp"
/*! Platform includes:
 *  This set of macros figures out which files to include based on your
 *  platform.
 */
#ifdef QT
#include "qt_serial_device.hpp"
#include "qt_thread.hpp"
#elif defined(__linux__)
//! handle array of characters
#include "linux_serial_device.hpp"
#include "posix_thread_manager.hpp"
#include <cstring>
#elif STM32
//! handle array of characters
#include <STM32F4DataGuard.h>
#include <STM32F4SerialDriver.h>
#include <stdlib.h>
#include <string.h>
#endif

namespace DJI
{
namespace OSDK
{

/****************************Globals**************************************/

#define MSG_ENABLE_FLAG_LEN 2

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------

typedef struct
{
  uint16_t sequence_number;
  uint8_t  session_id : 5;
  uint8_t  need_encrypt : 1;
  uint8_t  reserve : 2;
} req_id_t;

#define SET_CMD_SIZE (2u)

//----------------------------------------------------------------------
// Session Management
//----------------------------------------------------------------------

#define ACK_SESSION_IDLE 0
#define ACK_SESSION_PROCESS 1
#define ACK_SESSION_USING 2
#define CMD_SESSION_0 0
#define CMD_SESSION_1 1
#define CMD_SESSION_AUTO 32

#define POLL_TICK 20 // unit is ms

//----------------------------------------------------------------------
// Receive Management
//----------------------------------------------------------------------

typedef struct RecvContainer
{
  DJI::OSDK::ACK::Entry     recvInfo;
  DJI::OSDK::ACK::TypeUnion recvData;
  DJI::OSDK::DispatchInfo   dispatchInfo;
} RecvContainer;

//----------------------------------------------------------------------
// Codec Management
//----------------------------------------------------------------------

#define _SDK_U32_SET(_addr, _val) (*((uint32_t*)(_addr)) = (_val))
#define _SDK_U16_SET(_addr, _val) (*((uint16_t*)(_addr)) = (_val))

#define _SDK_CALC_CRC_HEAD(_msg, _len)                                         \
  sdk_stream_crc16_calc((const uint8_t*)(_msg), _len)
#define _SDK_CALC_CRC_TAIL(_msg, _len)                                         \
  sdk_stream_crc32_calc((const uint8_t*)(_msg), _len)

//----------------------------------------------------------------------
// CRC Management
//----------------------------------------------------------------------

const uint16_t crc_tab16[] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601,
  0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0,
  0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81,
  0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941,
  0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01,
  0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0,
  0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081,
  0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00,
  0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0,
  0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981,
  0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41,
  0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700,
  0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0,
  0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281,
  0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01,
  0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1,
  0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80,
  0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541,
  0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101,
  0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0,
  0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481,
  0x5440, 0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801,
  0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1,
  0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581,
  0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341,
  0x4100, 0x81c1, 0x8081, 0x4040
};

const uint32_t crc_tab32[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

const uint16_t CRC_INIT = 0x3AA3;

// const uint8_t encrypt = 0;
/*
#else
uint8_t encrypt = 0;
*/

//! @todo template Protocol for V1 V2 and SDK
//! more abstaction
class Protocol
{
public:
  //! Constructor
  Protocol(const char* device, uint32_t baudRate);

  //! Destructor
  ~Protocol()
  {
    delete (this->serialDevice);
  }

  /************************Public Interfaces**********************************/
  //! Send - callers are from above the ProtocolLayer

  /*
  void send(uint8_t session_mode, uint8_t is_enc, DJI_CONTROLLER_CMD
  cmd_set,
            uint8_t cmd_id, void *pdata, int len, bool isCallback, int
  callbackID,
      /** @note Compatible for DJI_APP_Pro_send
            int timeout = 0, int retry_time = 1);
  */
  void send(uint8_t session_mode, bool is_enc, const uint8_t cmd[], void* pdata,
            size_t len, int timeout = 0, int retry_time = 1,
            bool hasCallback = false, int callbackID = 0
            /** @note Better interface entrance*/
            );
  /** @note Main interface*/
  void send(Command* parameter);

  //! SendPoll:
  void sendPoll();

  /************************Receive Management********************************/

  RecvContainer receive();
  /************************Getters and setters*******************************/

  /**
   * Get serial device handler.
   */
  HardDriver* getDriver() const;

  /**
   * Get handler to thread data.
   */
  ThreadAbstract* getThreadHandle() const;

  /**********************************Fitlered******************************/
  void setKey(const char* key);

  /************************Useful frame-related constants*******************/
public:
  static const int     BUFFER_SIZE = 1024;
  static const int     ACK_SIZE    = 10;
  static const uint8_t SOF         = 0xAA;
  static const int     maxRecv     = BUFFER_SIZE;
  static const int     CRCHead     = sizeof(uint16_t);
  static const int     CRCData     = sizeof(uint32_t);
  static const int     CRCHeadLen  = sizeof(Header) - CRCHead;
  static const int     PackageMin  = sizeof(Header) + CRCData;
  uint8_t              buf[BUFFER_SIZE];

private:
  /***************************Init*******************************************/
  void init(HardDriver* Driver, MMU* mmuPtr, bool userCallbackThread = false);

  /***************************Receive Pipeline*******************************/

  typedef struct SDKFilter
  {
    uint16_t reuseIndex;
    uint16_t reuseCount;
    uint16_t recvIndex;
    uint8_t  recvBuf[BUFFER_SIZE];
    // for encrypt
    uint8_t sdkKey[32];
    uint8_t encode;
  } SDKFilter;

  //! Lowest-level function interfaces with SerialDevice
  bool readPoll(RecvContainer* allocatedRecvObject);

  //! Handle incoming data - byte level
  //! STM32 uses it directly
public:
  bool byteHandler(const uint8_t in_data, RecvContainer* allocatedRecvObject);
  //! Get the bufReadPos variable that tracks how much of the current serial buffer we have consumed
  int getBufReadPos();
  //! Get the readLen variable that tracks how many bytes were last read from the serialDevice
  int getReadLen();

private:
  //! Integrity checks for incoming data.
  bool streamHandler(SDKFilter* p_filter, uint8_t in_data,
                     RecvContainer* allocatedRecvObject);
  void storeData(SDKFilter* p_filter, uint8_t in_data);
  bool checkStream(SDKFilter* p_filter, RecvContainer* allocatedRecvObject);
  bool verifyHead(SDKFilter* p_filter, RecvContainer* allocatedRecvObject);
  bool verifyData(SDKFilter* p_filter, RecvContainer* allocatedRecvObject);

  //! Once checks are done, find out which branch of the receive pipeline to go
  //! to
  bool callApp(SDKFilter* p_filter, RecvContainer* allocatedRecvObject);

  //! For CMD-Frame data (push data) handling
  bool recvReqData(Header* protocolHeader, RecvContainer* allocatedRecvObject);

  //! A lot of ACK parsing logic is implemented here! It shouldn't be. @todo:
  //! Update the function
  bool appHandler(Header* protocolHeader, RecvContainer* allocatedRecvObject);

  //! CMD receive
  uint8_t getCmdCode(Header* protocolHeader);
  uint8_t getCmdSet(Header* protocolHeader);

  /*******************************Send Pipeline*****************************/

  int sendInterface(Command* cmdContainer);
  void sendData(uint8_t* buf);

  /****************************Multithreading support***********************/
  //! Thread sync for ACK
  ACK::TypeUnion allocateACK(Header* protocolHeader);

  void setACKFrameStatus(uint32_t usageFlag);

  /****************************Session Management***************************/

  void setup(void);
  void setupSession(void);

  void freeSession(CMDSession* session);
  CMDSession* allocSession(uint16_t session_id, uint16_t size);

  void freeACK(ACKSession* session);
  ACKSession* allocACK(uint16_t session_id, uint16_t size);

  /*******************************Encryption*******************************/
  uint16_t encrypt(uint8_t* pdest, const uint8_t* psrc, uint16_t w_len,
                   uint8_t is_ack, uint8_t is_enc, uint8_t session_id,
                   uint16_t seq_num);
  void encodeData(SDKFilter* p_filter, Header* p_head,
                  ptr_aes256_codec codec_func);

  /*******************************Utility Functions************************/
  uint16_t calculateLength(uint16_t size, uint16_t encrypt_flag);

  void transformTwoByte(const char* pstr, uint8_t* pdata);
  /***********************************CRC***********************************/
  void calculateCRC(void* p_data);
  uint16_t crc16_update(uint16_t crc, uint8_t ch);
  uint32_t crc32_update(uint32_t crc, uint8_t ch);
  uint16_t sdk_stream_crc16_calc(const uint8_t* pMsg, size_t nLen);
  uint32_t sdk_stream_crc32_calc(const uint8_t* pMsg, size_t nLen);
  void sdk_stream_prepare_lambda(SDKFilter* p_filter);
  void sdk_stream_shift_data_lambda(SDKFilter* p_filter);
  void sdk_stream_update_reuse_part_lambda(SDKFilter* p_filter);

private:
  /********************************Member variables*************************/

  //! Serial driver pointer
  HardDriver* serialDevice;

  //! Memory management
  MMU* mmu;

  //! Session Management
  CMDSession CMDSessionTab[SESSION_TABLE_NUM];
  ACKSession ACKSessionTab[SESSION_TABLE_NUM - 1];

  //! Serial filter
  SDKFilter filter;

  //! Encode buffers
  uint8_t encodeSendData[BUFFER_SIZE];
  uint8_t encodeACK[ACK_SIZE];

  //! Thread data
  bool            stopCond;
  ThreadAbstract* threadHandle;

  //! Frame-related.
  uint16_t seq_num;
  uint32_t ackFrameStatus;
  bool     broadcastFrameStatus;

  //! Buffer management

  int buf_read_pos;
  int read_len;
  int readPollCount;
};

} // namespace OSDK
} // namespace DJI

#endif // ONBOARDSDK_INTERNAL_DJI_PROTOCOLLAYER_H
