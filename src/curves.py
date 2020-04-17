#!/usr/bin/env python

import numpy as np


class Curve(object):
    def __init__(self, x_func, y_func):
        self.x_func = x_func
        self.y_func = y_func
        self._sampled_x = None
        self._sampled_y = None

    def sample(self, resolution=200):
        theta = np.linspace(0, 2 * np.pi, num=100, endpoint=False)
        self._sampled_x = self.x_func(theta)
        self._sampled_y = self.y_func(theta)
        return self._sampled_x, self._sampled_y
    
    def distance_to(self, x, y):
        # calculate the shortest distance between a point to this curve
        # return the distance and the coordinate of the point on the curve
        if not self._sampled_x and not self._sampled_y:
            self._sampled_x, self._sampled_y = self.sample()
        distance = np.square(self._sampled_x - x) + np.square(self._sampled_y - y)
        dis_min, idx_min = np.min(distance), np.argmin(distance)
        return dis_min, (self._sampled_x, self._sampled_y)


class Circle(Curve):
    def __init__(self, scale=(1, 1), bias=(0, 0)):
        self.x_func = lambda theta: scale[0] * np.sin(theta) + bias[0]
        self.y_func = lambda theta: scale[1] * np.cos(theta) + bias[1]
        self._sampled_x, self._sampled_y = None, None

    # override in the future for higher performance
    def distance_to(self, x, y):
        return super().distance_to(x, y)