#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump

class FiniteStateMachine:
	def __init__(self):
		rospy.init_node('fsm_node')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/bump', Bump, self.bump_callback)
		rospy.Subscriber('/scan', LaserScan, self.laser_callback)

		self.state = "forwards"
		self.bumped = False
		self.laser = 0.0
		self.turning_time = 0.0


	def bump_callback(self, data):
		if data.leftFront or data.rightFront:
			self.bumped = True
		else:
			self.bumped = False

	def laser_callback(self, data):
		self.laser = data.ranges[0]

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			velocity = Twist()
			if self.state == "forwards":
				if self.bumped:
					self.state = "backwards"
				else:
					velocity.linear.x = 0.1

			elif self.state == "backwards":
				if self.laser > 1.0:
					self.state = "left"
					self.turning_time = time.time()
				else:
					velocity.linear.x = -0.1

			elif self.state == "left":
				if time.time() - self.turning_time >= 1.0:
					self.state = "forwards"
				else:
					velocity.angular.z = 0.5

			self.pub.publish(velocity)

if __name__ == '__main__':
	try:
		robot = FiniteStateMachine()
		robot.run()
	except rospy.ROSInterruptException:
		pass