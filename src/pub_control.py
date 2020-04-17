#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    PoseStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler
from util import quaternion_to_angle


class Controller(object):
    def __init__(self):
        # Constants
        self.UPDATE_RATE = 20

        # Controller State
        self.car_pose = None

        # ----- Read Parameters -----
        control_topic = rospy.get_param(
            '~control_topic', '/mux/ackermann_cmd_mux/input/navigation')
        init_pose_topic = rospy.get_param(
            '~init_pose_topic', '/initialpose')
        car_pose_topic = rospy.get_param(
            '~car_pose_topic', '/car_pose')

        # ----- Publishers -----
        self.pub_controls = rospy.Publisher(
            control_topic, AckermannDriveStamped, queue_size=1)
        self.pub_init_pose = rospy.Publisher(
            init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

        # ----- Subscribers -----
        self.sub_car_pose = rospy.Subscriber(
            car_pose_topic, PoseStamped, self.car_pose_cb, queue_size=1)

        # ----- Timer -----
        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb)

    def car_pose_cb(self, msg):
        rotation = quaternion_to_angle(msg.pose.orientation)
        self.car_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            rotation
        ], dtype=np.float)
        print(self.car_pose)

    def timer_cb(self, event):
        print('---')

    def send_init_pose(self, pose_data):
        assert len(pose_data) == 3

        x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        point = Point(x=x, y=y)
        pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
        self.pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))


if __name__ == '__main__':
    rospy.init_node('pub_control')

    pid_controller = Controller()

    # ----- Main control loop -----
    rospy.spin()
