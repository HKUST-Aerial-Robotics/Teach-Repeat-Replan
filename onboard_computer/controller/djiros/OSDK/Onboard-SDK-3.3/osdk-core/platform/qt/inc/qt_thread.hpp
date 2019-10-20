#ifndef QT_THREAD
#define QT_THREAD

#include "dji_thread_manager.hpp"
#include <QMutex>
#include <QSerialPort>
#include <QThread>
#include <QWaitCondition>

// Let's not subclass OSDKThread from Qthread; this violates event loop
// constructs of Qt. Instead, let's moveToThread(OSDKThread).
class OSDKThread : public QObject, public DJI::OSDK::Thread
{
  Q_OBJECT

public:
  OSDKThread();
  OSDKThread(DJI::OSDK::Vehicle* vehicle, int Type, QObject* parent = 0);

  void callReceivePipeline();
  bool createThread();
  int  stopThread();

  size_t getCallTimes() const;
  void setCallTimes(const size_t& value);
  void setQThreadPtr(QThread* threadPtr)
  {
    this->qThreadPtr = threadPtr;
  }

private slots:
  void run();

private:
  QThread* qThreadPtr;
  int      numReceiveSignals;
  size_t   callTimes;
};

/*! @brief Qt-style Data Protection and Condition Variables for
 * Windows/unix/linux
 * platforms
 *
 */
class QThreadManager : public DJI::OSDK::ThreadAbstract
{
public:
  QThreadManager()
  {
  }
  ~QThreadManager();

  void init();

  //! Implementing virtual functions from ThreadManager
public:
  void lockMemory();
  void freeMemory();

  void lockMSG();
  void freeMSG();

  void lockACK();
  void freeACK();

  void lockProtocolHeader();
  void freeProtocolHeader();

  void lockNonBlockCBAck();
  void freeNonBlockCBAck();

  void lockStopCond();
  void freeStopCond();

  void lockFrame();
  void freeFrame();

  void notify();
  void notifyNonBlockCBAckRecv();
  void wait(int timeoutInSeconds);
  void nonBlockWait();

private:
  QMutex         m_memLock;
  QMutex         m_msgLock;
  QMutex         m_ackLock;
  QWaitCondition m_ackRecvCv;

  QMutex         m_headerLock;
  QMutex         m_nbAckLock;
  QWaitCondition m_nbAckRecv;

  //! Thread protection for setting stop condition for threads
  QMutex m_stopCondLock;

  //! Thread protection for last received frame storage
  QMutex m_frameLock;
};

#endif // QT_THREAD
