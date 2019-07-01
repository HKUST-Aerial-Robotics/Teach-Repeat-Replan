/*! @file Activate.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Activation process for the STM32 example App.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "Activate.h"

extern Vehicle  vehicle;
extern Vehicle* v;

void
userActivate()
{
  //! At your DJI developer account look for: app_key and app ID

  static char key_buf[65] = "your app_key here";

  DJI::OSDK::Vehicle::ActivateData user_act_data;
  user_act_data.ID = 0000; /*your app ID here*/

  user_act_data.encKey = key_buf;

  v->activate(&user_act_data);
}
