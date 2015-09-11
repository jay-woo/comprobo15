#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

import tty
import select
import sys
import termios

settings = termios.tcgetattr(sys.stdin)

""" Gets keypress information from the user """
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

""" Enables teleoperation of Neato using WASD controls"""
def teleop():
    # Sets up ROS node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleop_node')
    rate = rospy.Rate(10)
    key = None

    # Keeps publishing velocity values until shutdown
    while not rospy.is_shutdown():
        key = getKey()
        linear_vel = Vector3()
        angular_vel = Vector3()

        # Moves using WASD commands - stops if any other button is pressed
        if key == 'w':
            linear_vel.x = 0.5
        elif key == 'a':
            angular_vel.z = 1.0
        elif key == 's':
            linear_vel.x = -0.5
        elif key == 'd':
            angular_vel.z = -1.0

        velocity = Twist(linear_vel, angular_vel)
        pub.publish(velocity)
        rate.sleep()

        # Stops the program if CTRL + C is pressed
        if (key == '\x03'):
            break

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass