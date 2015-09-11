#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header

# Initializes ROS publisher
pub = rospy.Publisher('/my_sphere', Marker, queue_size=10)
rospy.init_node('sphere_publisher')

# Initializes the marker message's attributes
point_msg = Point(x=1.0, y=0.0)
pose_msg = Pose(position=point_msg)
scale_msg = Vector3(x=0.5, y=0.5, z=0.5)
color_msg = ColorRGBA(r=255, g=255, b=255, a=1.0)

# Repeatedly publishes the marker
r = rospy.Rate(10)
while not rospy.is_shutdown():
	header_msg = Header(stamp=rospy.Time.now(), frame_id="base_link")
	marker_msg = Marker(type=2, header=header_msg, scale=scale_msg, pose=pose_msg, color=color_msg

	pub.publish(marker_msg)
	r.sleep()