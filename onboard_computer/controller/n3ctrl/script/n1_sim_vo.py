#!/usr/bin/env python

import rospy
import numpy as np
import tf
import math
from tf import transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import Odometry as QuadOdometry
from std_msgs.msg import Header

EARTH_RADIUS = 6378137.0

odom_pub = None
br = None
wPz = None
wqz = None
kf_odom_msg = None
kfid = None
trigger = False
decimal = 0

def header_callback(msg):
    global trigger
    trigger = True

def msg_callback(odom_msg):
    w_q = np.array([odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w])
    w_p = np.array([odom_msg.pose.pose.position.x,
                    odom_msg.pose.pose.position.y,
                    odom_msg.pose.pose.position.z])

    w_v = np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.linear.y,
                    odom_msg.twist.twist.linear.z])

    wRb = tfs.quaternion_matrix(w_q)[0:3, 0:3]
    b_v = wRb.transpose().dot(w_v)

    global wPz
    global wqz
    global kf_odom_msg
    global kfid
    global trigger
    global decimal

    decimal += 1

    if trigger:
        trigger = False
        kfid += 1
        rospy.logwarn("Triggered #%d %f"%(kfid,odom_msg.header.stamp.to_sec()))
        #### set zero frame ####
        wPz = w_p
        wqz = w_q

        kf_odom_msg = Odometry()
        kf_odom_msg.header = odom_msg.header
        kf_odom_msg.header.frame_id = "zero"
        kf_odom_msg.child_frame_id = ""
        kf_odom_msg.pose.pose.orientation.x = 0
        kf_odom_msg.pose.pose.orientation.y = 0
        kf_odom_msg.pose.pose.orientation.z = 0
        kf_odom_msg.pose.pose.orientation.w = 1
        kf_odom_msg.pose.pose.position.x = 0
        kf_odom_msg.pose.pose.position.y = 0
        kf_odom_msg.pose.pose.position.z = 0
        kf_odom_msg.twist.twist.linear.x = b_v[0]
        kf_odom_msg.twist.twist.linear.y = b_v[1]
        kf_odom_msg.twist.twist.linear.z = b_v[2]

        odom_vo_msg = QuadOdometry()
        odom_vo_msg.status = QuadOdometry.STATUS_ODOM_VALID
        odom_vo_msg.kfid = kfid
        odom_vo_msg.kfodom = kf_odom_msg
        odom_vo_msg.curodom = kf_odom_msg
        odom_pub.publish(odom_vo_msg)

        return

    elif wPz is not None and wqz is not None and decimal%10==0:
        zRw = tfs.quaternion_matrix(wqz).transpose()[0:3, 0:3]
        z_p = zRw.dot(w_p - wPz)
        z_v = zRw.dot(w_v)
        z_q = tfs.quaternion_multiply(tfs.quaternion_inverse(wqz), w_q)

        #### generate current frame odometry ####
        cur_odom_msg = Odometry()
        cur_odom_msg.header = odom_msg.header
        cur_odom_msg.header.frame_id = "zero"
        cur_odom_msg.child_frame_id = ""
        cur_odom_msg.pose.pose.orientation.x = z_q[0]
        cur_odom_msg.pose.pose.orientation.y = z_q[1]
        cur_odom_msg.pose.pose.orientation.z = z_q[2]
        cur_odom_msg.pose.pose.orientation.w = z_q[3]
        cur_odom_msg.pose.pose.position.x = z_p[0]
        cur_odom_msg.pose.pose.position.y = z_p[1]
        cur_odom_msg.pose.pose.position.z = z_p[2]
        cur_odom_msg.twist.twist.linear.x = b_v[0]
        cur_odom_msg.twist.twist.linear.y = b_v[1]
        cur_odom_msg.twist.twist.linear.z = b_v[2]

        #### publish quadrotor_msgs::Odometry ####
        odom_vo_msg = QuadOdometry()
        odom_vo_msg.status = QuadOdometry.STATUS_ODOM_VALID
        odom_vo_msg.kfid = kfid
        odom_vo_msg.kfodom = kf_odom_msg
        odom_vo_msg.curodom = cur_odom_msg
        odom_pub.publish(odom_vo_msg)


        #### publish word-to-zero TF ####
        br.sendTransform((wPz[0], wPz[1], wPz[2]),
                         wqz,
                         odom_msg.header.stamp,
                         "zero",
                         "world")
        #### publish zero-to-zbody TF ####
        br.sendTransform((z_p[0], z_p[1], z_p[2]),
                         z_q,
                         odom_msg.header.stamp,
                         "zbody",
                         "zero")

if __name__ == "__main__":
    rospy.init_node('n3_sim_vo')

    odom_sub = rospy.Subscriber('~odom', Odometry, msg_callback)
    odom_sub = rospy.Subscriber('~trigger', Header, header_callback)
    odom_pub = rospy.Publisher('~odom_vo', QuadOdometry, queue_size=10)

    br = tf.TransformBroadcaster()

    kfid = 0

    rospy.spin()
