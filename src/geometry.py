#!/usr/bin/env python

import numpy as np


def calculate_theta(x, y):
    """ Extimate the theta. From -pi to pi. """
    square_sum = np.sqrt(np.square(x) + np.square(y))
    sin_theta = np.divide(y, square_sum)
    cos_theta = np.divide(x, square_sum)
    theta = np.zeros_like(square_sum)

    pos_cos = cos_theta >= 0
    theta[pos_cos] = np.arcsin(sin_theta[pos_cos])
    neg_cos_pos_sin = np.logical_and(cos_theta < 0, sin_theta >= 0)
    theta[neg_cos_pos_sin] = np.pi - np.arcsin(sin_theta[neg_cos_pos_sin])
    neg_cos_neg_sin = np.logical_and(cos_theta < 0, sin_theta < 0)
    theta[neg_cos_neg_sin] = -np.pi - np.arcsin(sin_theta[neg_cos_neg_sin])
 
    return theta


def calculate_diff_theta(theta1, theta2):
    diff = theta1 - theta2
    if diff > np.pi:
        diff = -diff + 2 * np.pi
    elif diff < -np.pi:
        diff = diff + 2 * np.pi
    diff = -diff
    return diff


class Drawer(object):
    def __init__(self, x_func, y_func):
        self.x_func = x_func
        self.y_func = y_func

    def draw(self, resolution=200):
        theta = np.linspace(0, 2 * np.pi, num=100, endpoint=False)
        sampled_x = self.x_func(theta)
        sampled_y = self.y_func(theta)
        return sampled_x, sampled_y


class Path(object):
    def __init__(self, xs, ys):
        self._x = np.asarray(xs)
        self._y = np.asarray(ys)
        self._theta = calculate_theta(xs, ys)
        self.resolution = len(xs)

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y
    
    def distance_to(self, x, y):
        # calculate the shortest distance between a point to this curve
        # return the distance and the coordinate of the point on the curve
        if self._x is None and self._y is None:
            self._x, self._y = self.sample()
        distance = np.square(self._x - x) + np.square(self._y - y)
        dis_min, idx_min = np.min(distance), np.argmin(distance)
        return dis_min, (self._x[idx_min], self._y[idx_min])

    def contains(self, x, y):
        theta_xy = calculate_theta(x, y)
        idx = np.argmin(np.abs(self._theta - theta_xy))
        origin_to_path = np.square(self._x[idx]) + np.square(self._y[idx]) 
        origin_to_xy = np.square(x) + np.square(y)
        return origin_to_xy < origin_to_path

    def estimate_derivative(self, theta):
        idx = np.argmin(np.abs(self._theta - theta))
        next_idx = idx + 1 if idx < self.resolution - 1 else 0
        last_idx = idx - 1 if idx > 0 else self.resolution - 1
        delta_x = self._x[last_idx] - self._x[next_idx]
        delta_y = self._y[last_idx] - self._y[next_idx]
        return calculate_theta(delta_x, delta_y)


class OvalPath(Path):
    def __init__(self, a, b, resolution=500):
        x_func = lambda t: a * np.cos(t)
        y_func = lambda t: b * np.sin(t)
        drawer = Drawer(x_func, y_func)
        super(OvalPath, self).__init__(*drawer.draw(resolution))


oval_path = OvalPath(5, 3, 500)
