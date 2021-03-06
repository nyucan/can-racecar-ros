#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry import oval_path, simple_track


def visualize_track(track):
    rospy.init_node('pub_marker')

    sampled_x, sampled_y = track.get_x(), track.get_y()

    # create a publisher
    pub_visualize = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(30)

    points = Marker()
    points.header.frame_id = '/map'
    points.header.stamp = rospy.Time.now()
    points.ns = 'path'
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS

    points.scale.x, points.scale.y = 0.05, 0.05

    points.color.a = 1.0
    points.color.r = 1.0
    points.color.g = 0.0
    points.color.b = 0.0
    
    # Create a list of points
    for x, y in zip(sampled_x, sampled_y):
        p = Point()
        p.x, p.y, p.z = x, y, 0
        points.points.append(p)

    while not rospy.is_shutdown():
        pub_visualize.publish(points)
        rate.sleep()


if __name__ == '__main__':
    visualize_track(simple_track)
