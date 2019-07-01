#include "CameraGimbalSample.h"

using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;

bool
gimbalCameraControl()
{
  int               responseTimeout = 0;
  ACK::ErrorCode    ack;
  GimbalContainer   gimbal;
  RotationAngle     initialAngle;
  RotationAngle     currentAngle;
  Gimbal::SpeedData gimbalSpeed;

  // Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
  int pkgIndex = 0;
  int freq     = 10;

  if (v->getFwVersion() != Version::M100_31)
  {
    Telemetry::TopicName topicList10Hz[] = { Telemetry::TOPIC_GIMBAL_ANGLES,
                                             Telemetry::TOPIC_GIMBAL_STATUS };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = v->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }

    v->subscribe->startPackage(pkgIndex);
    delay_nms(500);
    /*	ack = waitForACK();
            if (ACK::getError(ack) != ACK::SUCCESS)
            {
                    ACK::getErrorCodeMessage(ack, __func__);
                    // Cleanup before return
                    v->subscribe->removePackage(pkgIndex);
                    return false;
            }
    */
    // Wait for data to arrive
    delay_nms(2000);
  }

  printf("Please note that the gimbal yaw angle you see in the telemetry is "
         "w.r.t absolute North"
         ", and the accuracy depends on your magnetometer calibration.\n\n");

  // Get Gimbal initial values
  if (v->getFwVersion() != Version::M100_31)
  {
    initialAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    initialAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    initialAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    initialAngle.roll  = v->broadcast->getGimbal().roll;
    initialAngle.pitch = v->broadcast->getGimbal().pitch;
    initialAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  printf("Initial Gimbal rotation angle: [ %0.3f, %0.3f, %0.3f ]\n\n",
         initialAngle.roll, initialAngle.pitch, initialAngle.yaw);

  // Re-set Gimbal to initial values
  gimbal = GimbalContainer(0, 0, 0, 20, 1, false, false, false, initialAngle);
  doSetGimbalAngle(&gimbal);

  printf("Setting new Gimbal rotation angle to [0,20,180] using "
         "incremental control:\n");

  // Get current gimbal data to calc precision error in post processing
  if (v->getFwVersion() != Version::M100_31)
  {
    currentAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = v->broadcast->getGimbal().roll;
    currentAngle.pitch = v->broadcast->getGimbal().pitch;
    currentAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  gimbal = GimbalContainer(0, 200, 1800, 20, 0, false, false, false,
                           initialAngle, currentAngle);
  doSetGimbalAngle(&gimbal);

  if (v->getFwVersion() != Version::M100_31)
  {
    currentAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = v->broadcast->getGimbal().roll;
    currentAngle.pitch = v->broadcast->getGimbal().pitch;
    currentAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);

  // Take picture
  printf("Ensure SD card is present.\n");
  printf("Taking picture..\n");
  v->camera->shootPhoto();
  printf("Check DJI GO App or SD card for a new picture.\n");

  printf("Setting new Gimbal rotation angle to [0,-50, 0] using absolute "
         "control:\n");
  gimbal =
    GimbalContainer(0, -500, 0, 20, 1, false, false, false, initialAngle);
  doSetGimbalAngle(&gimbal);

  if (v->getFwVersion() != Version::M100_31)
  {
    currentAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = v->broadcast->getGimbal().roll;
    currentAngle.pitch = v->broadcast->getGimbal().pitch;
    currentAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);

  // Start video: We will keep the video doing for the duration of the speed
  // control.
  printf("Ensure SD card is present.\n");
  printf("Starting video..\n");
  v->camera->videoStart();

  // Speed control

  printf("Gimbal Speed Description: \n\n"
         "Roll - unit 0.1 degrees/second input rate [-1800, 1800]\n"
         "Pitch - unit 0.1 degrees/second input rate [-1800, 1800]\n"
         "Yaw - unit 0.1 degrees/second input rate [-1800, 1800]\n\n");

  printf("Setting Roll rate to 10, Pitch rate to 5, Yaw Rate to -20.\n");
  gimbalSpeed.roll  = 100;
  gimbalSpeed.pitch = 50;
  gimbalSpeed.yaw   = -200;

  int speedControlDurationMs = 4000;
  int incrementMs            = 100;
  for (int timer = 0; timer < speedControlDurationMs; timer += incrementMs)
  {
    v->gimbal->setSpeed(&gimbalSpeed);
    // usleep(incrementMs * 1000);
    delay_nms(incrementMs);
  }

  if (v->getFwVersion() != Version::M100_31)
  {
    currentAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = v->broadcast->getGimbal().roll;
    currentAngle.pitch = v->broadcast->getGimbal().pitch;
    currentAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);

  // Reset the position
  printf("Resetting position...\n");
  gimbal = GimbalContainer(0, 0, 0, 20, 1, false, false, false, initialAngle);
  doSetGimbalAngle(&gimbal);

  if (v->getFwVersion() != Version::M100_31)
  {
    currentAngle.roll =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw =
      v->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = v->broadcast->getGimbal().roll;
    currentAngle.pitch = v->broadcast->getGimbal().pitch;
    currentAngle.yaw   = v->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);

  // Stop the video
  printf("Stopping video...\n");
  v->camera->videoStop();
  delay_nms(1000);
  printf("Check DJI GO App or SD card for a new video.\n");

  // Cleanup and exit gimbal sample
  if (v->getFwVersion() != Version::M100_31)
  {
    v->subscribe->removePackage(pkgIndex);
    delay_nms(3000);
    /*ack = waitForAck();
            if (ACK::getError(ack))
            {
                    printf("Error unsubscribing; please restart the drone/FC to
       get back "
                                                                     "to a clean
       state.\n");
            }
    */
  }
  return true;
}

void
doSetGimbalAngle(GimbalContainer* gimbal)
{
  Gimbal::AngleData gimbalAngle;
  gimbalAngle.roll     = gimbal->roll;
  gimbalAngle.pitch    = gimbal->pitch;
  gimbalAngle.yaw      = gimbal->yaw;
  gimbalAngle.duration = gimbal->duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal->isAbsolute;
  gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

  v->gimbal->setAngle(&gimbalAngle);
  // Give time for gimbal to sync
  delay_nms(4000);
}

void
displayResult(RotationAngle* currentAngle)
{
  printf("New Gimbal rotation angle is [ %0.3f, %0.3f, %0.3f ]\n\n",
         currentAngle->roll, currentAngle->pitch, currentAngle->yaw);
}
