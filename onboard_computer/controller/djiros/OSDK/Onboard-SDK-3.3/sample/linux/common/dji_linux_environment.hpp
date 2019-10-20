/*! @file dji_linux_environment.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef ONBOARDSDK_DJI_ENVIRONMENT_H
#define ONBOARDSDK_DJI_ENVIRONMENT_H

#include <fstream>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <unistd.h>

class DJI_Environment
{
public:
  DJI_Environment(const std::string& config_file_path);
  ~DJI_Environment();

  static std::string findFile(std::string file);
  int                getApp_id() const;
  const std::string& getEnc_key() const;
  const std::string& getDevice() const;
  unsigned int       getBaudrate() const;
  bool               getConfigResult() const;
  bool parse(std::string config_file_path);

private:
  std::string  config_file_path;
  int          app_id;
  std::string  enc_key;
  std::string  device;
  unsigned int baudrate;
  bool         config_read_result;
};

#endif // ONBOARDSDK_DJI_ENVIRONMENT_H
