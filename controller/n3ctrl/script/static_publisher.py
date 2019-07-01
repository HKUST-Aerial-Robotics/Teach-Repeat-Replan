#!/usr/bin/env python

import rospy
import tf
from tf import transformations as tfs
from math import pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

if __name__ == "__main__":
    rospy.init_node('static_publisher')
    rate = rospy.Rate(100.0)

    imu_pub = rospy.Publisher('~imu', Imu, queue_size=10)
    joy_pub = rospy.Publisher('~joy', Joy, queue_size=10)
    odom_pub = rospy.Publisher('~odom', Odometry, queue_size=10)

    count = 0
    while not rospy.is_shutdown():
        rate.sleep()
        if count % 2 == 0:
            # 50Hz
            # imu
            q = tfs.quaternion_from_euler(
                30.0 / 180.0 * pi,
                0.0 / 180.0 * pi,
                10.0 / 180.0 * pi,
                'rzyx')
            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            msg.linear_acceleration.x = 0.5
            msg.linear_acceleration.y = 0.5
            msg.linear_acceleration.z = 0.5

            imu_pub.publish(msg)

            # joy
            # msg = Joy()
            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "body"
            # msg.axes = [0.2, 0.2, 0.2, 0.0]
            # joy_pub.publish(msg)

        if count % 1 == 0:
            # 100Hz
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            q = tfs.quaternion_from_euler(
                30.0 / 180.0 * pi,
                0.0 / 180.0 * pi,
                10.0 / 180.0 * pi,
                'rzyx')

            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            msg.pose.pose.position.x = 0
            msg.pose.pose.position.y = 0
            msg.pose.pose.position.z = 1

            msg.twist.twist.linear.x = 0
            msg.twist.twist.linear.y = 0
            msg.twist.twist.linear.z = 0

            odom_pub.publish(msg)

            br = tf.TransformBroadcaster()
            br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                             q,
                             rospy.Time.now(),
                             "base_link",
                             "world")

            br = tf.TransformBroadcaster()
            e = tfs.euler_from_quaternion(q, 'rzyx')
            qc = tfs.quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')
            br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                             qc,
                             rospy.Time.now(),
                             "intermediate",
                             "world")

        count += 1
