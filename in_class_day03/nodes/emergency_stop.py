#!/usr/bin/env python
import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3

class EmergencyStop:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.init_node('emergency_stop_node')
        rate = rospy.Rate(10)

        self.bump = [0, 0, 0, 0]

        while not rospy.is_shutdown():
            linear_vel = Vector3(x=0.3, y=0, z=0)
            for i in self.bump:
                if i:
                    linear_vel.x = 0.0

            self.pub.publish(Twist(linear=linear_vel))

            rate.sleep()


    def bump_callback(self, data):
        self.bump = [data.leftFront, data.rightFront, data.leftSide, data.rightSide]

if __name__ == '__main__':
    try:
        robot = EmergencyStop()
    except rospy.ROSInterruptException:
        pass