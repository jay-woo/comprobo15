#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs import Pose, Position, Point
from std_msgs import ColorRGBA, Header

pub = rospy.Publisher('/my_sphere', Marker, queue_size=10)

rospy.init_node('sphere_publisher')

header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
point_msg = Point(x=1.0, y=2.0)
pose_msg = Pose(position=point_msg)
color_msg = ColorRGBA(r=255, g=255, b=255, a=1.0)
marker_msg = Marker(type=2, header=header_msg pose=pose_msg, color=color_msg)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	pub.publish(marker_msg)
	r.sleep()