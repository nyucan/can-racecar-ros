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
from curves import standard_circle


class Controller(object):
    def __init__(self, track, init_pose):
        # Constants
        self.UPDATE_RATE = 20.0

        # Controller State
        self.car_pose = None
        self.track = track

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

        # initialization
        rospy.sleep(rospy.Duration.from_sec(0.5))
        self.send_init_pose(init_pose)

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

    def timer_cb(self, event):
        self.calculcate_diff_drive()

    def send_init_pose(self, pose_data):
        assert len(pose_data) == 3
        x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        point = Point(x=x, y=y)
        pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
        self.pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

    def send_command(self, cmd):
        assert len(cmd) == 2
        v, delta = float(cmd[0]), float(cmd[1])
        drive = AckermannDrive(steering_angle=delta, speed=v)
        self.pub_controls.publish(AckermannDriveStamped(drive=drive))

    def transform_to_theta(self, x, y):
        """ Extimate theta. From -pi to pi. """
        theta = 0
        if x == 0 and y == 0:
            return theta
        square_sum = np.sqrt(np.square(x) + np.square(y))
        sin_theta = y / square_sum
        cos_theta = x / square_sum
        if cos_theta >= 0:
            theta = np.arcsin(sin_theta)
        elif sin_theta >= 0:
            theta = np.pi - np.arcsin(sin_theta)
        else:
            theta = -np.pi - np.arcsin(sin_theta)
        return theta

    def estimate_deritive(self, theta, curve):
        delta = 0.0001
        delta_x = curve.x_func(theta + delta) - curve.x_func(theta - delta)
        delta_y = curve.y_func(theta + delta) - curve.y_func(theta - delta)
        return self.transform_to_theta(delta_x, delta_y)

    def calculcate_diff_drive(self):
        car_x, car_y, car_ori = self.car_pose
        theta = self.transform_to_theta(car_x, car_y)
        radius_sq = self.track.x_func(theta) ** 2 + self.track.y_func(theta) ** 2
        diff_dis, track_point = self.track.distance_to(car_x, car_y)
        track_x, track_y = track_point

        # calculate diff distance
        if radius_sq >= car_x ** 2 + car_y ** 2:
            diff_dis = -diff_dis

        # calculate diff orientation
        track_theta = self.transform_to_theta(track_x, track_y)
        track_ori = self.estimate_deritive(track_theta, self.track)
        diff_ori = car_ori - track_ori
        if diff_ori > np.pi:
            diff_ori = -diff_ori + 2 * np.pi
        elif diff_ori < -np.pi:
            diff_ori = diff_ori + 2 * np.pi

        print(diff_dis, diff_ori) 


if __name__ == '__main__':
    rospy.init_node('pub_control')
    circle = standard_circle

    pid_controller = Controller(
        track=standard_circle, init_pose=(0, 3, 0))

    # ----- Main control loop -----
    rospy.spin()
