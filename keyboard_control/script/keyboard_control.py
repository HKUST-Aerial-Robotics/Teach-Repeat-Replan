import pygame
from pygame.locals import *
import time
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def main():

    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 150, 150)
    screen = pygame.display.set_mode(window_size.size)

    # initialize ros publisher
    twist_pub = rospy.Publisher('keyboard/twist', Twist, queue_size=10)
    joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
    rospy.init_node('keyboard_control')
    rate = rospy.Rate(10)

    _twist = Twist()
    vel = 1.0
    ang = 2.0
    while not rospy.is_shutdown():
        rate.sleep()
        # get key value and set Twist
        for event in pygame.event.get():
            # key down to set velocity
            if event.type == KEYDOWN:

                # space to pause to UAV
                if event.key == pygame.K_SPACE:
                    print 'stop'

                # x,y velocity control: x axis forward, y axis right
                elif event.key == pygame.K_UP:
                    print 'forward'
                    _twist.linear.x = vel
                elif event.key == pygame.K_DOWN:
                    print 'backward'
                    _twist.linear.x = -vel
                elif event.key == pygame.K_LEFT:
                    print 'left'
                    _twist.linear.y = vel
                elif event.key == pygame.K_RIGHT:
                    print 'right'
                    _twist.linear.y = -vel

                # yaw and z control
                elif event.key == pygame.K_w:
                    print 'up'
                    _twist.linear.z = vel
                elif event.key == pygame.K_s:
                    print 'down'
                    _twist.linear.z = -vel
                elif event.key == pygame.K_a:
                    print 'turn left'
                    _twist.angular.z = ang
                elif event.key == pygame.K_d:
                    print 'turn right'
                    _twist.angular.z = -ang
                elif event.key == pygame.K_ESCAPE:
                    sys.exit()

                twist_pub.publish(_twist)

            # when keyup, reset velcity
            elif event.type == pygame.KEYUP:
                # twist = Twist()
                # _twist = Twist()
                # x,y velocity control: x axis forward, y axis right
                if event.key == pygame.K_UP:
                    print 'forward'
                    _twist.linear.x = 0.0
                elif event.key == pygame.K_DOWN:
                    print 'backward'
                    _twist.linear.x = 0.0
                elif event.key == pygame.K_LEFT:
                    print 'left'
                    _twist.linear.y = 0.0
                elif event.key == pygame.K_RIGHT:
                    print 'right'
                    _twist.linear.y = 0.0

                # yaw and z control
                elif event.key == pygame.K_w:
                    print 'up'
                    _twist.linear.z = 0.0
                elif event.key == pygame.K_s:
                    print 'down'
                    _twist.linear.z = 0.0
                elif event.key == pygame.K_a:
                    print 'turn left'
                    _twist.angular.z = 0.0
                elif event.key == pygame.K_d:
                    print 'turn right'
                    _twist.angular.z = 0.0
                twist_pub.publish(_twist)
                print 'stop'


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
