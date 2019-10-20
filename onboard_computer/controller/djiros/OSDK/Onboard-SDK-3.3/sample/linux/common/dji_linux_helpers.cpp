//
// Created by rohit on 5/12/17.
//

#include <dji_linux_helpers.hpp>

using namespace DJI::OSDK;

Vehicle*
setupOSDK(int argc, char** argv)
{

  int functionTimeout = 1;

  // Config file loading
  std::string config_file_path;
  if (argc > 1)
  {
    config_file_path = argv[1];
    std::ifstream fStream(config_file_path.c_str());
    if (!fStream.good())
      throw std::runtime_error("User configuration file not found");
  }
  else
  {
    config_file_path = DJI_Environment::findFile("UserConfig.txt");

    if (config_file_path.empty())
      throw std::runtime_error("User configuration file not found");
  }

  DJI_Environment* environment = new DJI_Environment(config_file_path);
  if (!environment->getConfigResult())
  {
    // We were unable to read the config file. Exit.
    return NULL;
  }
/*  LinuxSerialDevice* linuxSerialDevice = new LinuxSerialDevice(
    environment->getDevice().c_str(), environment->getBaudrate());
  linuxSerialDevice->init();
  bool setupStatus = validateSerialDevice(linuxSerialDevice);

  if (!setupStatus)
  {
    delete (linuxSerialDevice);
    delete (environment);
    return NULL;
  }
  else
  {
    delete (linuxSerialDevice);
  }
*/
  bool     threadSupport = true;
  Vehicle* vehicle       = new Vehicle(environment->getDevice().c_str(),
                                 environment->getBaudrate(), threadSupport);

  // Check if the communication is working fine
  if (!vehicle->protocolLayer->getDriver()->getDeviceStatus())
  {
    std::cout << "Comms appear to be incorrectly set up. Exiting.\n";
    delete (vehicle);
    delete (environment);
    return NULL;
  }

  // Check if drone version is okay
  if (vehicle->getFwVersion() < extendedVersionBase &&
    vehicle->getFwVersion() != Version::M100_31)
  {
    return NULL;
  }

  // Activate
  Vehicle::ActivateData activateData;
  activateData.ID = environment->getApp_id();
  char app_key[65];
  activateData.encKey = app_key;
  strcpy(activateData.encKey, environment->getEnc_key().c_str());
  activateData.version = vehicle->getFwVersion();
  ACK::ErrorCode ack   = vehicle->activate(&activateData, functionTimeout);

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    delete (vehicle);
    delete (environment);
    return NULL;
  }

  delete (environment); // We have no more use for environment
  return vehicle;
}

bool
validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;

  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead())
  {
    DERROR(
      "Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if (serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    DERROR("Succeeded to read from serial device\n");
  }
  else
  {
    DERROR("\"Failed to read from serial device. The Onboard SDK is not "
           "communicating with your drone. \n");
    // serialDevice->unsetSerialPureTimedRead();
    return (false);
  }

  // If we reach here, _serialRead succeeded.
  int baudCheckStatus = serialDevice->checkBaudRate(buf);
  if (baudCheckStatus == -1)
  {
    DERROR("No data on the line. Is your drone powered on?\n");
    return false;
  }
  if (baudCheckStatus == -2)
  {
    DERROR("Baud rate mismatch found. Make sure DJI Assistant 2 has the same "
           "baud setting as the one in User_Config.h\n");
    return (false);
  }
  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}
