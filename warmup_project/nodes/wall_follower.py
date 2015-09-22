#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

""" Commands the Neato to follow a nearby wall """
class WallFollower(object):

	""" Initializes ROS and other internal variables """
	def __init__(self):
		# Defines ROS variables
		rospy.init_node('wall_follower_node')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

		# Saves scan data
		self.scan = [0.] * 360

		self.turning = False
		self.turn_speed = 0.0

	""" Callback function for the LIDAR scanner """
	def scan_callback(self, data):
		self.scan = data.ranges

	""" Publishes velocity data to the Neato """
	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Finds the closest wall to follow (ignores south side)
			rospy.wait_for_message('/scan', LaserScan, timeout=10)
			wall_scans = {'n': self.scan[0], 'w': self.scan[90], 'e': self.scan[270]}
			min_val = 100
			for i in wall_scans:
				if wall_scans[i] < min_val and wall_scans[i] != 0.0:
					min_val = wall_scans[i]
					closest_wall = i

			velocity = Twist()

			# If facing north, turns until there is leeway to move forwards
			if closest_wall == 'n' or (wall_scans['n'] <= 1.0 and wall_scans['n'] != 0.0):
				if not self.turning:
					self.turn_speed = -0.3
					if wall_scans['w'] > wall_scans['e'] and wall_scans['e'] != 0.0:
						self.turn_speed = 0.3

				self.turning = True
				velocity.angular.z = self.turn_speed

			# Otherwise, follows the east or west facing wall
			else:
				self.turning = False
				velocity.linear.x = 0.1

				error = 0
				if closest_wall == 'w':
					error = self.scan[45] - self.scan[135]
				elif closest_wall == 'e':
					error = self.scan[225] - self.scan[315]

				velocity.angular.z = min(0.2 * error, 0.5)

				print error

			self.pub.publish(velocity)
			r.sleep()


if __name__ == '__main__':
	try:
		robot = WallFollower()
		robot.run()
	except rospy.ROSInterruptException:
		pass