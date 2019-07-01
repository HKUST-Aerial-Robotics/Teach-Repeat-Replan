#!/usr/bin/env python

import rospy
import numpy as np
import tf
import math
import message_filters
from tf import transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from quadrotor_msgs.msg import Odometry as QuadOdometry

EARTH_RADIUS = 6378137.0

odom_pub = None
br = None
origin_gps = None
gP0 = None
gqw = None


def msg_callback(imu_msg, velo_msg, gps_msg):
    global origin_gps
    if origin_gps is None:
        origin_gps = np.array([gps_msg.latitude,
                               gps_msg.longitude,
                               gps_msg.altitude])
        rospy.loginfo("Origin is [%f,%f,%f]" % (math.degrees(origin_gps[0]),
                                                math.degrees(origin_gps[1]),
                                                origin_gps[2]))
        return

    now_gps = np.array([gps_msg.latitude / 180.0 * math.pi,
                        gps_msg.longitude / 180.0 * math.pi,
                        gps_msg.altitude])

    assert(gps_msg.header.frame_id == "NED")
    g_p = np.array([+(now_gps[0] - origin_gps[0]) * EARTH_RADIUS,
                    -(now_gps[1] - origin_gps[1]) * EARTH_RADIUS,
                    +(now_gps[2] - origin_gps[2])])

    assert(imu_msg.header.frame_id == "FLU")
    g_q = np.array([imu_msg.orientation.x,
                    imu_msg.orientation.y,
                    imu_msg.orientation.z,
                    imu_msg.orientation.w])
    w = np.array([imu_msg.angular_velocity.x,
                  imu_msg.angular_velocity.y,
                  imu_msg.angular_velocity.z])

    assert(velo_msg.header.frame_id == "NED")
    g_v = np.array([velo_msg.vector.x,
                    -velo_msg.vector.y,
                    -velo_msg.vector.z])

    global gP0
    global gqw

    if gP0 is None and gqw is None:
        gP0 = g_p
        e = tfs.euler_from_quaternion(g_q, 'rzyx')
        gqw = tfs.quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

        return
    else:
        wRg = tfs.quaternion_matrix(gqw).transpose()[0:3,0:3]
        w_p = wRg.dot(g_p - gP0)
        w_v = wRg.dot(g_v)
        w_q = tfs.quaternion_multiply(tfs.quaternion_inverse(gqw),g_q)

    #### odom ####
    odom_msg = Odometry()
    odom_msg.header = imu_msg.header
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = ""
    odom_msg.pose.pose.orientation.x = w_q[0]
    odom_msg.pose.pose.orientation.y = w_q[1]
    odom_msg.pose.pose.orientation.z = w_q[2]
    odom_msg.pose.pose.orientation.w = w_q[3]
    odom_msg.pose.pose.position.x = w_p[0]
    odom_msg.pose.pose.position.y = w_p[1]
    odom_msg.pose.pose.position.z = w_p[2]
    odom_msg.twist.twist.linear.x = w_v[0]
    odom_msg.twist.twist.linear.y = w_v[1]
    odom_msg.twist.twist.linear.z = w_v[2]
    odom_msg.twist.twist.angular.x = w[0]
    odom_msg.twist.twist.angular.y = w[1]
    odom_msg.twist.twist.angular.z = w[2]
    odom_pub.publish(odom_msg)

    e = tfs.euler_from_quaternion(w_q, 'rzyx')
    wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
    wqc = tfs.quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

    #### tf ####

    br.sendTransform((w_p[0], w_p[1], w_p[2]),
                     wqb,
                     odom_msg.header.stamp,
                     "body",
                     "world")

    br.sendTransform(((w_p[0], w_p[1], w_p[2])),
                     wqc,
                     odom_msg.header.stamp,
                     "intermediate",
                     "world")

if __name__ == "__main__":
    rospy.init_node('n3_sim_helper')

    imu_sub = message_filters.Subscriber('~imu', Imu)
    velo_sub = message_filters.Subscriber('~velo', Vector3Stamped)
    gps_sub = message_filters.Subscriber('~gps', NavSatFix)

    ts = message_filters.TimeSynchronizer([imu_sub, velo_sub, gps_sub], 10)
    ts.registerCallback(msg_callback)

    odom_pub = rospy.Publisher('~odom', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    rospy.spin()
