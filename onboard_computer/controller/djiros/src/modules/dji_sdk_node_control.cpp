/** @file dji_sdk_node_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the control functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>
#include <iostream> 

/*!
 * @brief The flight controller takes control signals with frame convention:
 *   body:   Forward-Right for horizontal
 *           CW = + for yaw
 *           U  = + for z
 *   ground: North-East for horizontal,
 *           CW = + for yaw ,
 *           U  = + for z ,
 *
 *  However, the telemetry data in our node follows REP 103:
 *    body: FLU
 *    ground: ENU
 *  The client code is encouraged to follow the REP 103, which means that
 *  the control signal the client generates is FLU or ENU, and needs to be
 *  transformed to the convention of the flight controller.
 */
void DJISDKNode::flightControl(uint8_t flag, float xSP, float ySP, float zSP, float yawSP)
{

  uint8_t HORI  = (flag & 0xC0);
  uint8_t VERT  = (flag & 0x30);
  uint8_t YAW   = (flag & 0x08);
  uint8_t FRAME = (flag & 0x06);
  uint8_t HOLD  = (flag & 0x01);

  double xCmd, yCmd, zCmd, yawCmd;
  if (FRAME == Control::HORIZONTAL_GROUND)
  {
    // 1.1 Horizontal channels
    if ( (HORI == Control::HORIZONTAL_VELOCITY) || (HORI == Control::HORIZONTAL_POSITION) )
    {
      xCmd = ySP;
      yCmd = xSP;
    }
    else
    {
      ROS_DEBUG("GROUND frame is specified, but angle and rate command is generated in body frame");
      xCmd = RAD2DEG(xSP);
      yCmd = RAD2DEG(-ySP);
    }

    // 1.2 Verticle Channel
    if ( (VERT == Control::VERTICAL_VELOCITY) || (VERT == Control::VERTICAL_POSITION) )
    {
      zCmd = zSP;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "GROUND frame is specified, but thrust command is generated in body frame");
      zCmd = zSP;
    }
  }
  else if(FRAME == Control::HORIZONTAL_BODY)
  {
    // 2.1 Horizontal channels
    if ( (HORI == Control::HORIZONTAL_VELOCITY) || (HORI == Control::HORIZONTAL_POSITION) )
    {
      // The X and Y Vel and Pos should be only based on rotation after Yaw,
      // whithout roll and pitch. Otherwise the behavior will be weird.

      // Transform from F-R to F-L
      xCmd = xSP;
      yCmd = -ySP;
    }
    else
    {
      xCmd = RAD2DEG(xSP);
      yCmd = RAD2DEG(-ySP);
    }

    // 2.2 Vertical channel
    if ( (VERT == Control::VERTICAL_VELOCITY) || (VERT == Control::VERTICAL_POSITION)  )
    {
      ROS_WARN_THROTTLE(1.0, "BODY frame is specified, but hight and z-velocity is generated in ground frame");
      zCmd = zSP;
    }
    else
    {
      zCmd = zSP;
    }
  }

  // The behavior of yaw should be the same in either frame
  if ( YAW == Control::YAW_ANGLE )
  {
    tf::Matrix3x3 rotationSrc;
    rotationSrc.setRPY(0.0, 0.0, yawSP);

    //The last term should be transpose, but since it's symmetric ...
    tf::Matrix3x3 rotationDes (R_ENU2NED * rotationSrc * R_FLU2FRD);

    double temp1, temp2;
    rotationDes.getRPY(temp1, temp2, yawCmd);

    yawCmd = RAD2DEG(yawCmd);
  }
  else if (YAW == Control::YAW_RATE)
  {
    yawCmd = RAD2DEG(-yawSP);
  }

  Control::CtrlData ctrlData(flag, xCmd, yCmd, zCmd, yawCmd);
  vehicle->control->flightCtrl(ctrlData);
}

void
DJISDKNode::flightControlSetpointCallback(
  const sensor_msgs::Joy::ConstPtr& pMsg)
{ 
  float xSP    = pMsg->axes[0];
  float ySP    = pMsg->axes[1];
  float zSP    = pMsg->axes[2];
  float yawSP  = pMsg->axes[3];
  uint8_t flag = (uint8_t)(pMsg->axes[4]);

  flightControl(flag, xSP, ySP, zSP, yawSP);
}

void
DJISDKNode::flightControlPxPyPzYawCallback(
  const sensor_msgs::Joy::ConstPtr& pMsg)
{
  uint8_t flag = (Control::VERTICAL_POSITION |
                  Control::HORIZONTAL_POSITION |
                  Control::YAW_ANGLE |
                  Control::HORIZONTAL_GROUND |
                  Control::STABLE_ENABLE);

  float px    = pMsg->axes[0];
  float py    = pMsg->axes[1];
  float pz    = pMsg->axes[2];
  float yaw   = pMsg->axes[3];

  flightControl(flag, px, py, pz, yaw);
}

void
DJISDKNode::flightControlVxVyVzYawrateCallback(
  const sensor_msgs::Joy::ConstPtr& pMsg)
{
  uint8_t flag = (Control::VERTICAL_VELOCITY |
                  Control::HORIZONTAL_VELOCITY |
                  Control::YAW_RATE |
                  Control::HORIZONTAL_GROUND |
                  Control::STABLE_ENABLE);
  float vx        = pMsg->axes[0];
  float vy        = pMsg->axes[1];
  float vz        = pMsg->axes[2];
  float yawRate   = pMsg->axes[3];

  flightControl(flag, vx, vy, vz, yawRate);
}

void
DJISDKNode::flightControlRollPitchPzYawrateCallback(
  const sensor_msgs::Joy::ConstPtr& pMsg)
{
  uint8_t flag = (Control::VERTICAL_POSITION |
                  Control::HORIZONTAL_ANGLE |
                  Control::YAW_RATE |
                  Control::HORIZONTAL_BODY |
                  Control::STABLE_ENABLE);

  float roll      = pMsg->axes[0];
  float pitch     = pMsg->axes[1];
  float pz        = pMsg->axes[2];
  float yawRate   = pMsg->axes[3];

  flightControl(flag, roll, pitch, pz, yawRate);
}

