#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy


imu_pub  = None
odom_pub = None
br = None

def sim_odom_callback(sim_odom_msg):
    q = np.array([sim_odom_msg.pose.pose.orientation.x,
                  sim_odom_msg.pose.pose.orientation.y,
                  sim_odom_msg.pose.pose.orientation.z,
                  sim_odom_msg.pose.pose.orientation.w])
    p = np.array([sim_odom_msg.pose.pose.position.x,
                  sim_odom_msg.pose.pose.position.y,
                  sim_odom_msg.pose.pose.position.z])

    e = tfs.euler_from_quaternion(q, 'rzyx')
    wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
    wqc = tfs.quaternion_from_euler(e[0],  0.0,  0.0, 'rzyx')

    #### imu ####
    # imu yaw drift in simulation 
    #imu_q = tfs.quaternion_from_euler(e[0]+30.0/180.0*pi, e[1], e[2], 'rzyx')
    imu_q = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
    imu_msg = Imu()
    imu_msg.header.stamp = sim_odom_msg.header.stamp
    imu_msg.header.frame_id = "base_link"
    imu_msg.orientation.x = imu_q[0]
    imu_msg.orientation.y = imu_q[1]
    imu_msg.orientation.z = imu_q[2]
    imu_msg.orientation.w = imu_q[3]

    imu_pub.publish(imu_msg)

    #### odom ####
    odom_msg = sim_odom_msg
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = ""
    odom_pub.publish(odom_msg)

    #### tf ####
    
    br.sendTransform((p[0], p[1], p[2]),
                     wqb,
                     sim_odom_msg.header.stamp,
                     "base_link",
                     "world")

    br.sendTransform(((p[0], p[1], p[2])),
                     wqc,
                     sim_odom_msg.header.stamp,
                     "intermediate",
                     "world")


if __name__ == "__main__":
    rospy.init_node('sim_helper')

    sim_odom_sub = rospy.Subscriber('~sim_odom', Odometry, sim_odom_callback)
    
    imu_pub = rospy.Publisher('~imu', Imu, queue_size=10)
    odom_pub = rospy.Publisher('~odom', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    rospy.spin();
