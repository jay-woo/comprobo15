#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class DistanceEmergencyStop:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.init_node('emergency_stop_node')
        rate = rospy.Rate(10)

        self.lasers = []

        while not rospy.is_shutdown():
            linear_vel = Vector3(x=0.1, y=0, z=0)
            for i in self.lasers:
                if i < 1.0 and i != 0.0:
                    linear_vel.x = 0.0

            self.pub.publish(Twist(linear=linear_vel))

            rate.sleep()


    def laser_callback(self, data):
        self.lasers = data.ranges

if __name__ == '__main__':
    try:
        robot = DistanceEmergencyStop()
    except rospy.ROSInterruptException:
        pass