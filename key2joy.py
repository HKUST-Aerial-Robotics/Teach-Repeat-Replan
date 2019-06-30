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
    window_size = Rect(0, 0, 691, 272)
    screen = pygame.display.set_mode(window_size.size)

    img = pygame.image.load("./files/keyboard3.jpg")

    # initialize ros publisher
    twist_pub = rospy.Publisher('keyboard/twist', Twist, queue_size=10)
    joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
    rospy.init_node('key2joy')
    rate = rospy.Rate(50)

    # init joy message
    joy_ = Joy()    
    joy_.header.frame_id = 'map'
    for i in range(8):
      joy_.axes.append(0.0)
    for i in range(11):
      joy_.buttons.append(0)

    while not rospy.is_shutdown():
        rate.sleep()
        screen.blit(img, (1,1))
        pygame.display.flip()
        # reset message axes
#        for i in range(8):
#          joy_.axes[i] = 0.0
#
#        # reset buttons
#        for i in range(11):
#          joy_.buttons[i] = 0

        for event in pygame.event.get():
            # ---------------------- key dowm message ----------------------
            
            if event.type == KEYDOWN:
                # position control
                if event.key == pygame.K_UP:
                    print 'forward'
                    joy_.axes[4] = 1.0
                if event.key == pygame.K_DOWN:
                    print 'backward'
                    joy_.axes[4] = -1.0
                if event.key == pygame.K_LEFT:
                    print 'left'
                    joy_.axes[3] = 1.0
                if event.key == pygame.K_RIGHT:
                    print 'right'
                    joy_.axes[3] = -1.0
                # yaw and z control
                if event.key == pygame.K_w:
                    print 'up'
                    joy_.axes[1] = 1.0
                if event.key == pygame.K_s:
                    print 'down'
                    joy_.axes[1] = -1.0
                if event.key == pygame.K_a:
                    print 'turn left'
                    joy_.axes[0] = 1.0
                if event.key == pygame.K_d:
                    print 'turn right'
                    joy_.axes[0] = -1.0
                # task control
                if event.key == pygame.K_n:
                    print 'clear'
                    joy_.buttons[6] = 1
                if event.key == pygame.K_m:
                    print 'start'
                    joy_.buttons[7] = 1
                joy_pub.publish(joy_)

            # when keyup, reset velcity
            elif event.type == pygame.KEYUP:
                # position control
                if event.key == pygame.K_UP:
                    # print 'forward'
                    joy_.axes[4] = 0.0
                if event.key == pygame.K_DOWN:
                    # print 'backward'
                    joy_.axes[4] = -0.0
                if event.key == pygame.K_LEFT:
                    # print 'left'
                    joy_.axes[3] = 0.0
                if event.key == pygame.K_RIGHT:
                    # print 'right'
                    joy_.axes[3] = -0.0
                # yaw and z control
                if event.key == pygame.K_w:
                    # print 'up'
                    joy_.axes[1] = 0.0
                if event.key == pygame.K_s:
                    # print 'down'
                    joy_.axes[1] = -0.0
                if event.key == pygame.K_a:
                    # print 'turn left'
                    joy_.axes[0] = 0.0
                if event.key == pygame.K_d:
                    # print 'turn right'
                    joy_.axes[0] = -0.0
                # task control
                if event.key == pygame.K_n:
                    # print 'start'
                    joy_.buttons[6] = 0
                if event.key == pygame.K_m:
                    # print 'clear'
                    joy_.buttons[7] = 0
                joy_pub.publish(joy_)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
