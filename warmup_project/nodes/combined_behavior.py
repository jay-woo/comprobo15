#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry

from math import pi, cos, sin, atan, atan2, sqrt
from tf.transformations import euler_from_quaternion

""" Robot avoids obstacles, until it sees something directly in front of it.
	Then the robot will follow the target until the target disappears. """
class CombinedBehavior(object):
	def __init__(self):
		rospy.init_node('combined_behavior_node')
		self.pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
		
		# Saves the laser scan as both a list (for obstacle avoidance) and as a dictionary (for object tracking)
		self.scan = [0.] * 360
		self.scan_dict = {}

		self.pose = Pose()
		self.connected_components = []
		self.mode = 0

	""" Stores laser scan data from the robot """
	def laser_callback(self, data):
		#### Saves the scan data into a list ####
		self.scan = data.ranges[:360]

		#### Saves the scan data into a dictionary ####
		# Stores scan values for the range -30 degrees to 30 degrees
		angles = range(330, 360)      # Stores angles from 330 -> 0 -> 30
		angles.extend(range(0, 31))

		values = []
		values.extend(data.ranges[330:360])
		values.extend(data.ranges[0:31])

		self.scan_dict = dict(zip(angles, values))  # Stores angles and respective values in a dictionary

		# Keeps track of connected components (each component = list of angles)
		self.connected_components = []
		component = []
		threshold = 0  # If objects are disconnected by a few indexes, connect them into one object
		for i in angles:
			if self.scan_dict[i] != 0.0:
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

	""" Stores odometry data from the robot """
	def odom_callback(self, data):
		self.pose = data.pose.pose

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

	""" Determines if an object is in front of the Neato """
	def front_detected(self):
		for i in range(0, 15):
			if self.scan[i] != 0.0 and self.scan[i] < 2.0:
				return True

		for i in range(345, 360):
			if self.scan[i] != 0.0 and self.scan[i] < 2.0:
				return True

	""" Avoids obstacles using a force field """
	def avoid_obstacles(self):
		force_x = 0.0
		force_y = 0.0
		robot_x, robot_y, robot_yaw = convert_pose_to_xy_and_theta(self.pose)

		# Calculates the x and y components of the vector, for each scan item
		for i in range(360):
			if self.scan[i] == 0.0:
				continue

			theta = i * pi / 180.0
			x = self.scan[i] * cos(theta)
			y = self.scan[i] * sin(theta)
			dist = sqrt(x**2 + y**2)

			x_hat = -kx * cos(theta) / dist**2
			y_hat = -ky * sin(theta) / dist**2

			force_x += x_hat
			force_y += y_hat

		# Points the Neato in the direction of the force
		error_yaw = atan2(force_y, force_x)
		if error_yaw < 0:
			yaw = max(2.0 * error_yaw, -0.5)
		else:
			yaw = min(2.0 * error_yaw, 0.5)

		# Only moves the Neato if the distance is large enough
		error_dist = sqrt(force_x**2 + force_y**2)
		if error_dist > 0.5 and abs(error_yaw) < 0.05:
			throttle = min(0.5 * error_dist, 0.3)

		# Stops the Neato completely if there is nothing to avoid
		if error_dist <= 0.5:
			throttle = 0
			yaw = 0

		return throttle, yaw

	""" Tracks an object directly in front of the Neato """
	def track_target(self):
		throttle, yaw = 0, 0

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
			yaw = 1.0 * theta_error

			# Moves forwards if the object is further away
			if y_track > 1.0:
				throttle = 0.2

		return throttle, yaw

	""" Main loop """
	def run(self):
		r = rospy.Rate(10)
		kx = 1e-2 # x component scaling factor for LIDAR
		ky = 1e-2 # y component scaling factor for LIDAR

		while not rospy.is_shutdown():
			if self.front_detected():
				self.mode = 1
			else:
				self.mode = 0

			velocity = Twist()

			# If there is no obstacle in front of the Neato, avoid obstacles
			if self.mode == 0:
				velocity.linear.x, velocity.angular.z = self.avoid_obstacles()

			# If an object is in front of the Neato, track it
			else:
				velocity.linear.x, velocity.angular.z = self.track_target()

			self.pub_velocity.publish(velocity)
			r.sleep()


""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
def convert_pose_to_xy_and_theta(pose):
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

if __name__ == '__main__':
	try:
		robot = CombinedBehavior()
		robot.run()
	except rospy.ROSInterruptException:
		pass