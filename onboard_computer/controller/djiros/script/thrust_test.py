#!/usr/bin/python

import rospy
from sensor_msgs.msg import Imu, Joy
import numpy as np
from tf import transformations as tfs
import math
import random

VERT_THRUST = 1.0
VERT_VELO = -1.0
GEAR_SHIFT_VALUE = -0.6
ROLL_PITCH_SCALE = 30.0
YAW_SCALE = 60.0
YAW_MODE_RATE = 1.0

def angle_add(a,b):
    y = a + b
    if (y > math.pi):
        y -= math.pi * 2
    if (y < -math.pi):
        y += math.pi * 2
    return y

class ThrustTest:

    def __init__(self):
        self.w_q_b = None
        self.rc_data = None
        self.des_thrust = 30.0
        self.ctrl_pub = None
        self.yaw = None

    def rc_callback(self, msg):
        self.rc_data = dict(roll=msg.axes[0],
                            pitch=msg.axes[1],
                            yaw=msg.axes[2],
                            thrust=msg.axes[3],
                            mode=msg.axes[4],
                            gear=msg.axes[5])

        self.process()

    def imu_callback(self, msg):
        self.w_q_b = np.array([msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z,
                               msg.orientation.w])

    def process(self):
        if self.rc_data is None or self.w_q_b is None:
            return

        eulers = tfs.euler_from_quaternion(self.w_q_b, 'rzyx')
        yaw = eulers[0]

        des_roll = self.rc_data['roll'] * ROLL_PITCH_SCALE
        des_pitch = -self.rc_data['pitch'] * ROLL_PITCH_SCALE
        des_yawrate = self.rc_data['yaw'] * YAW_SCALE
        if (abs(des_yawrate) < 1.0):
            des_yawrate = 0.0

        # normalized to 0 ~ 1
        thr_from_rc = (self.rc_data['thrust'] + 1.0) / 2.0
        # map to 10~100
        thr_from_rc = 10.0 + 90.0 * thr_from_rc

        if (self.rc_data['gear'] < GEAR_SHIFT_VALUE):
            pass
        else:
            self.des_thrust = thr_from_rc

        rospy.loginfo("rc[{:.2f}] rcmap[{: 3.0f}] ctrl[{: 3.0f}] yawrate[{:.2f}]".format(
            self.rc_data['thrust'], thr_from_rc, self.des_thrust,des_yawrate))

        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "FRD"
        joy_msg.axes = [des_roll, des_pitch, self.des_thrust,
                        des_yawrate, VERT_THRUST, YAW_MODE_RATE]
        self.ctrl_pub.publish(joy_msg)

if __name__ == "__main__":
    rospy.init_node('thrust_test')

    thrust_test = ThrustTest()

    rc_sub = rospy.Subscriber('/djiros/rc', Joy, thrust_test.rc_callback)
    imu_sub = rospy.Subscriber('/djiros/imu', Imu, thrust_test.imu_callback)

    thrust_test.ctrl_pub = rospy.Publisher('/djiros/ctrl', Joy, queue_size=10)
    rospy.spin()
