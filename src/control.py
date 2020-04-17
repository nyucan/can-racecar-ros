#!/usr/bin/env python

import math
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    PoseStamped,
    Quaternion,
)
from util import quaternion_to_angle


def car_pose_cb(msg):
    rotation = quaternion_to_angle(msg.pose.orientation)
    arr_car_pose = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        rotation
    ], dtype=np.float)
    print(arr_car_pose)


def path_following_controller(arr_car_pose, trajectory):
    # the PID controller for trajectory tracking
    pass


def circle_generater():
    r = np.arange(360)
    x_scale, y_scale = 3, 3
    xs, ys = x_scale * np.sin(r), y_scale * np.cos(r)
    return np.stack((xs, ys), axis=1)


def create_marker_demo():
    rospy.init_node('control')

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

    points.scale.x = 0.05
    points.scale.y = 0.05

    points.color.a = 1.0
    points.color.r = 1.0
    points.color.g = 0.0
    points.color.b = 0.0
    
    # Create a list of points
    for x, y in circle_generater():
        p = Point()
        p.x, p.y, p.z = x, y, 0
        points.points.append(p)


    while not rospy.is_shutdown():
        pub_visualize.publish(points)
        rate.sleep()


def main():
    rospy.init_node('control')

    # ----- Publishers -----
    # control_topic = rospy.get_param("~control_topic", "/mux/ackermann_cmd_mux/input/navigation")
    # pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    # init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    # pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    # ----- Subscribers -----
    car_pose_topic = rospy.get_param('~car_pose_topic', '/car_pose')
    sub_car_pose = rospy.Subscriber(car_pose_topic, PoseStamped, car_pose_cb, queue_size=1)

    dur = rospy.Duration(5.0)
    rate = rospy.Rate(10)
    start = rospy.Time.now()

    # main control loop
    while rospy.Time.now() - start < dur:
        rate.sleep()


if __name__ == "__main__":
    # main()
    create_marker_demo()
