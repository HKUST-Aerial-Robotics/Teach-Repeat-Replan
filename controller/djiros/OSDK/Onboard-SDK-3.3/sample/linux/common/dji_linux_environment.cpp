/*! @file dji_linux_environment.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "dji_linux_environment.hpp"

DJI_Environment::DJI_Environment(const std::string& config_file_path)
{

  this->config_file_path   = config_file_path;
  this->config_read_result = this->parse(config_file_path);
}
DJI_Environment::~DJI_Environment()
{
}

/**
 * @note Find file within osdk-core directory
 */
std::string
DJI_Environment::findFile(std::string file)
{
  char        cwd[1024];
  std::string configFile;

  if (getcwd(cwd, sizeof(cwd)) == NULL)
    throw std::runtime_error("Error getting current directory");

  std::string strCWD(cwd);
  // configFile = strCWD + "/osdk-core/" + file;
  configFile = strCWD + "/" + file; // just in the current working directory

  std::ifstream fStream(configFile.c_str());

  if (!fStream.good())
    configFile.clear();

  return configFile;
}

int
DJI_Environment::getApp_id() const
{
  return app_id;
}

const std::string&
DJI_Environment::getEnc_key() const
{
  return enc_key;
}

const std::string&
DJI_Environment::getDevice() const
{
  return device;
}

unsigned int
DJI_Environment::getBaudrate() const
{
  return baudrate;
}

bool
DJI_Environment::getConfigResult() const
{
  return config_read_result;
}

bool
DJI_Environment::parse(std::string config_file_path)
{
  char        line[1024];
  static char key[70];
  char        devName[20];
  int         id;

  bool setID = false, setKey = false, setBaud = false, setSerialDevice = false;
  bool result = false;

  std::ifstream read(config_file_path);

  if (read.is_open())
  {
    while (!read.eof())
    {
      read.getline(line, 1024);
      if (*line != 0) //! @note sscanf have features on empty buffer.
      {
        if (sscanf(line, "app_id : %d", &this->app_id))
        {
          std::cout << "Read App ID\n";
          setID = true;
        }
        if (sscanf(line, "app_key : %s", key))
        {
          this->enc_key = std::string(key);
          setKey        = true;
        }
        if (sscanf(line, "device : %s", devName))
        {
          this->device    = std::string(devName);
          setSerialDevice = true;
        }
        if (sscanf(line, "baudrate : %d", &this->baudrate))
        {
          setBaud = true;
        }
      }
    }
    if (setBaud && setID && setKey && setSerialDevice)
    {
      std::cout << "User Configuration read successfully. \n\n";
      result = true;
    }
    else
    {
      std::cout << "There's an error with your UserConfig.txt file.\n";
      result = false;
    }

    read.close();
  }
  else
  {
    std::cout << "User config file could not be opened. Make sure your "
                 "filepath is correct\n"
              << "and have sufficient permissions." << std::endl;
    result = false;
  }

  return result;
}
