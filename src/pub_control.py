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
from geometry import oval_path
from geometry import calculate_theta, calculate_diff_theta


class Controller(object):
    def __init__(self, track, init_pose):
        # Constants
        self.UPDATE_RATE = 20.0
        self.MAX_STEERING_ANGLE = 0.34
        self.MAX_VELOCITY = 2.0

        # Controller State
        self.car_pose = None
        self.track = track
        self.last_diff = None

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
        rospy.sleep(rospy.Duration.from_sec(3))
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
        self.apply_pid_controller()

    def apply_pid_controller(self, model='PD'):
        velocity = 0.4 * self.MAX_VELOCITY
        steering_angle = 0.0
        if model == 'P':
            diff = np.array(self.calculcate_diff_drive(*self.car_pose))
            K = np.array([1, 0.1])
            err_str = 'Error(dis, psi): {:.2f}, {:.2f}'.format(*diff)
        elif model == 'PD':
            diff = np.array(self.calculate_pd_diff_drive(*self.car_pose))
            K = np.array([1, 0.1, 0.1, 0.1])
            err_str = 'Error(dis, psi, dot_dis, dot_psi): {:.2f}, {:.2f}, {:.2f}, {:.2f}'.format(*diff)
        elif model == 'PID':
            raise NotImplementedError
        else:
            raise RuntimeError('Choose a model from \'P\', \'PD\' and \'PID\'')
        steering_angle = np.clip(K.dot(diff), -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        print('{} Steering: {:2f}'.format(err_str, steering_angle))
        self.send_command((velocity, steering_angle))

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

    def estimate_future_pose(self, epslion=0.1):
        car_x, car_y, car_ori = self.car_pose
        est_x = car_x + epslion * np.cos(car_ori)
        est_y = car_y + epslion * np.sin(car_ori)
        return est_x, est_y, car_ori

    def calculcate_diff_drive(self, x, y, psi):
        """ Calculate (d - d_des) and (psi - psi_des) for a proportional controller. """
        diff_dis, track_point = self.track.distance_to(x, y)
        if not self.track.contains(x, y):
            diff_dis = -diff_dis

        # calculate diff orientation
        psi_des = self.track.estimate_derivative(calculate_theta(*track_point))
        diff_psi = calculate_diff_theta(psi, psi_des)

        return diff_dis, diff_psi

    def calculate_pd_diff_drive(self, x, y, psi):
        diff_dis, diff_psi = self.calculcate_diff_drive(x, y, psi)
        if self.last_diff is None:
            dot_diff_dis, dot_diff_psi = 0, 0
        else:
            dot_diff_dis = (diff_dis - self.last_diff[0]) * self.UPDATE_RATE
            dot_diff_psi = calculate_diff_theta(diff_psi, self.last_diff[1]) * self.UPDATE_RATE
        self.last_diff = (diff_dis, diff_psi) 
        return diff_dis, diff_psi, dot_diff_dis, dot_diff_psi


if __name__ == '__main__':
    rospy.init_node('pub_control')

    pid_controller = Controller(
        track=oval_path, init_pose=(0, 3, 0))

    # ----- Main control loop -----
    rospy.spin()
