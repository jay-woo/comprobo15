#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math

### HELPER FUNCTIONS ###

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

def distance(point1, point2):
    """ Returns the distance between two points """
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

########################

""" Commands the robot to drive in a 1m x 1m square continuously """
class DriveSquare:

    def __init__(self):
        """ Initializes all the important variables. Also contains main ROS loop. """

        # Sets up ROS node and publishers/subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.init_node('square_node')
        rate = rospy.Rate(10)

        # Robot's current position
        self.xy = [0.0, 0.0]
        self.theta = 0.0

        # Robot's previous position (before and after a turn)
        self.xy_prev = [self.xy[0], self.xy[1]]
        self.theta_prev = 0.0

        self.turning = False

        while not rospy.is_shutdown():
            linear_vel = Vector3()
            angular_vel = Vector3()

            # Moves forwards 1m, then turns pi/2 radians
            if not self.turning:
                linear_vel.x = 0.3

                if distance(self.xy, self.xy_prev) >= 1.0:
                    linear_vel.x = 0.0
                    self.theta_prev = self.theta
                    self.turning = True

            else:
                angular_vel.z = -0.5
                theta_diff = abs(self.theta - self.theta_prev)

                if theta_diff >= (math.pi / 2):
                    angular_vel.z = 0.0
                    self.xy_prev = [self.xy[0], self.xy[1]]
                    self.turning = False

            velocity = Twist(linear_vel, angular_vel)
            self.pub.publish(velocity)
            rate.sleep()

    def odom_callback(self, data):
        """ Records the Neato's current odometry reading """
        pose = data.pose.pose
        (self.xy[0], self.xy[1], self.theta) = convert_pose_to_xy_and_theta(pose)

if __name__ == '__main__':
    try:
        robot = DriveSquare()
    except rospy.ROSInterruptException:
        pass