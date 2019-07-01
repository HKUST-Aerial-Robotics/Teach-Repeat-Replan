#include <dji_vehicle.hpp>
#include <qt_thread.hpp>

using namespace DJI::OSDK;

OSDKThread::OSDKThread(DJI::OSDK::Vehicle* vehicle, int Type, QObject* parent)
{
  this->vehicle = vehicle;
  type          = Type;
  callTimes     = 0;
  this->vehicle->setStopCond(false);
  numReceiveSignals = 0;
}

void
OSDKThread::run()
{
  while (!(vehicle->getStopCond()))
  {
    callTimes++;
    if (type == 1)
      vehicle->protocolLayer->sendPoll();
    else if (type == 2)
    {
      callReceivePipeline();
    }
    else if (type == 3)
    {
      vehicle->callbackPoll();
      QThread::msleep(1);
    }
    QThread::usleep(100);
  }
}

void
OSDKThread::callReceivePipeline()
{
  int numProcess = 0;
  while (numProcess == 0 || numReceiveSignals == 0 ||
         (vehicle->protocolLayer->getBufReadPos() <
          vehicle->protocolLayer->getReadLen()))
  {
    numProcess++;
    if (vehicle->getStopCond())
      break;
    RecvContainer recvFrame = vehicle->protocolLayer->receive();
    if (recvFrame.recvInfo.cmd_id != 0xFF)
    {
      vehicle->processReceivedData(recvFrame);
    }
    if (numReceiveSignals == 0)
      numReceiveSignals++;
  }
  numReceiveSignals++;
}
bool
OSDKThread::createThread()
{
  //  this->start();
  //  return this->isRunning();
  return true;
}

int
OSDKThread::stopThread()
{
  vehicle->setStopCond(true);
  qThreadPtr->quit();
  return 0;
}

size_t
OSDKThread::getCallTimes() const
{
  return callTimes;
}

void
OSDKThread::setCallTimes(const size_t& value)
{
  callTimes = value;
}

OSDKThread::OSDKThread()
{
  vehicle = 0;
  type    = 0;
}

QThreadManager::~QThreadManager()
{
}

void
QThreadManager::init()
{
}

void
QThreadManager::lockMemory()
{
  m_memLock.lock();
}

void
QThreadManager::freeMemory()
{
  m_memLock.unlock();
}

void
QThreadManager::lockMSG()
{
  m_msgLock.lock();
}

void
QThreadManager::freeMSG()
{
  m_msgLock.unlock();
}

void
QThreadManager::lockACK()
{
  m_ackLock.lock();
}

void
QThreadManager::freeACK()
{
  m_ackLock.unlock();
}

void
QThreadManager::lockProtocolHeader()
{
  m_headerLock.lock();
}

void
QThreadManager::freeProtocolHeader()
{
  m_headerLock.unlock();
}

void
QThreadManager::lockNonBlockCBAck()
{
  m_nbAckLock.lock();
}

void
QThreadManager::freeNonBlockCBAck()
{
  m_nbAckLock.unlock();
}

void
QThreadManager::lockStopCond()
{
  m_stopCondLock.lock();
}

void
QThreadManager::freeStopCond()
{
  m_stopCondLock.unlock();
}

void
QThreadManager::lockFrame()
{
  m_frameLock.lock();
}

void
QThreadManager::freeFrame()
{
  m_frameLock.unlock();
}

void
QThreadManager::notify()
{
  lockACK();
  m_ackRecvCv.wakeAll();
  freeACK();
}

void
QThreadManager::notifyNonBlockCBAckRecv()
{
  m_nbAckRecv.wakeAll();
}

void
QThreadManager::nonBlockWait()
{
  m_nbAckRecv.wait(&m_nbAckLock);
}

void
QThreadManager::wait(int timeoutInSeconds)
{
  unsigned long timeout_ms = 1000 * timeoutInSeconds;
  m_ackRecvCv.wait(&m_ackLock, timeout_ms);
}
