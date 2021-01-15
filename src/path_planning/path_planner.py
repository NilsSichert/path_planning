from __future__ import print_function
import numpy as np
import threading


class PathPlanner(object):
    def __init__(self, waypoints):
        self.test = 0
        self.waypoints = waypoints

    def get_target(self, current_waypoint, current_position):
        pass
