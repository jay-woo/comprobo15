#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class WallApproach(object):
	def __init__(self):
		rospy.init_node('wall_approach_node')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

		self.current_distance = 0.0

	def laser_callback(self, data):
		self.current_distance = data.ranges[0]

	def run(self):
		target_distance = rospy.get_param('~target_distance')

		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			velocity = Twist(linear=Vector3(x=0.2))

			if self.current_distance != 0.0:
				error = self.current_distance - target_distance
				velocity.linear.x = 0.5 * error

			self.pub.publish(velocity)
			print velocity.linear.x

			r.sleep()

if __name__ == '__main__':
	try:
		robot = WallApproach()
		robot.run()
	except rospy.ROSInterruptException:
		pass