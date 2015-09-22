#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

from math import sin, cos, atan, pi, sqrt

""" Commands the robot to roughly follow a moving person """
class PersonFollower(object):
	""" Initializes ROS and other internal variables """
	def __init__(self):
		rospy.init_node('person_follower_node')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

		self.scan = {}
		self.connected_components = []

	""" Collects and parses through laser data """
	def laser_callback(self, data):
		# Stores scan values for the range -30 degrees to 30 degrees
		angles = range(330, 360)      # Stores angles from 330 -> 0 -> 30
		angles.extend(range(0, 31))

		values = []
		values.extend(data.ranges[330:360])
		values.extend(data.ranges[0:31])

		self.scan = dict(zip(angles, values))  # Stores angles and respective values in a dictionary

		# Keeps track of connected components (each component = list of angles)
		self.connected_components = []
		component = []
		threshold = 0  # If objects are disconnected by a few indexes, connect them into one object
		for i in angles:
			if self.scan[i] != 0.0:
				threshold = 0
				component.append(i)
			else:
				threshold += 1
				if threshold <= 10:
					continue
				elif component != []:
					self.connected_components.append(component)
					threshold = 0
					component = []

		# Adds last component, if it missed one
		if component != []:
			self.connected_components.append(component)

	""" Calculates the center of mass of a connected component """
	def center_of_mass(self, component):
		com_x, com_y = 0.0, 0.0

		for i in component:
			r = self.scan[i]
			theta = i * pi / 180.0
			com_x += r * sin(theta)
			com_y += r * cos(theta)

		com_x /= len(component)
		com_y /= len(component)

		return com_x, com_y

	""" Tracks the connected component that is closest to the Neato """
	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			velocity = Twist()

			if self.connected_components:
				smallest_distance = 100
				x_trax, y_track = 0.0, 0.0

				# Finds the closest object to track
				for i in self.connected_components:
					(x, y) = self.center_of_mass(i)
					distance = sqrt(x**2 + y**2)

					if distance < smallest_distance:
						smallest_distance = 100
						x_track, y_track = x, y

				# Rotates Neato towards object
				theta_error = atan(x_track / y_track)
				velocity.angular.z = 1.0 * theta_error

				# Moves forwards if the object is further away
				if y_track > 1.0:
					velocity.linear.x = 0.2

			self.pub.publish(velocity)

			r.sleep()

if __name__ == '__main__':
	try:
		robot = PersonFollower()
		robot.run()
	except rospy.ROSInterruptException:
		pass