#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry

from math import pi, cos, sin, atan2, sqrt
from tf.transformations import euler_from_quaternion

""" Commands the robot to reach a certain goal, while also avoiding nearby objects """
class ObstacleAvoider(object):
	def __init__(self):
		rospy.init_node('obstacle_avoider_node')
		self.pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
		
		self.scan = [0.] * 360
		self.pose = Pose()

	""" Stores laser scan data from the robot """
	def laser_callback(self, data):
		self.scan = data.ranges[:360]

	""" Stores odometry data from the robot """
	def odom_callback(self, data):
		self.pose = data.pose.pose

	""" Main loop """
	def run(self):
		r = rospy.Rate(10)
		kx = 1e-1 # x component scaling factor for LIDAR
		ky = 1e-1 # y component scaling factor for LIDAR

		while not rospy.is_shutdown():
			velocity = Twist()
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
			target_yaw = atan2(force_y, force_x)

			error_yaw = target_yaw
			if error_yaw < 0:
				velocity.angular.z = max(2.0 * error_yaw, -0.5)
			else:
				velocity.angular.z = min(2.0 * error_yaw, 0.5)
 
			# Only moves the Neato if the distance is large enough
			error_dist = sqrt(force_x**2 + force_y**2)
			if error_dist > 0.5 and abs(error_yaw) < 0.05:
				velocity.linear.x = min(0.5 * error_dist, 0.3)

			self.pub_velocity.publish(velocity)
			r.sleep()

""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
def convert_pose_to_xy_and_theta(pose):
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

if __name__ == '__main__':
	try:
		robot = ObstacleAvoider()
		robot.run()
	except rospy.ROSInterruptException:
		pass