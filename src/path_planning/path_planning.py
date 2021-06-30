import numpy
import rospy


class Path(object):
    def __init__(self):
        self._target_index = 0
        self._waypoints = numpy.zeros([0, 3], dtype=numpy.float)
        self._distances = numpy.zeros_like(self._waypoints)
        self._target_point = numpy.zeros([3], dtype=numpy.float)

    def update_target(self,
                      position,
                      look_ahead_distance,
                      loop=True,
                      ignore_z=False):
        look_ahead_square = look_ahead_distance**2
        n_points = self._waypoints.shape[0]
        index = self._target_index
        success = False
        for _ in range(n_points):
            if index >= n_points:
                index = 0
                if not loop:
                    break
            if ignore_z:
                vec = self._waypoints[index, :2] - position[:2]
            else:
                vec = self._waypoints[index], position
            square = numpy.inner(vec, vec)
            if square > look_ahead_square:
                success = True
                break
            index += 1
        if not success:
            if not loop:
                index = n_points - 1

        self._target_index = index % n_points
        self._target_point = self._waypoints[self._target_index]
        return success

    def get_distance_between_points(self, from_index, to_index):
        if to_index < from_index:
            dist = numpy.sum(self._distances[from_index:])
            dist += numpy.sum(self._distances[:to_index])
        else:
            dist = numpy.sum(self._distances[from_index:to_index])
        return dist

    def get_distance_to_point(self, index):
        return self.get_distance_between_points(self._target_index, index)

    def get_target_point(self):
        return numpy.copy(self._target_point)

    def update_path_from_param_server(self, name="~path"):
        path_param = rospy.get_param(name)
        self._waypoints = self.get_points_from_yaml_list(path_param)
        n_waypoints = numpy.shape(self._waypoints)[0]
        self._distances = numpy.zeros([n_waypoints])
        for i in range(0, n_waypoints - 1):
            diff = self._waypoints[i + 1] - self._waypoints[i]
            self._distances[i] = numpy.sqrt(diff[0]**2 + diff[1]**2 +
                                            diff[2]**2)
        diff = self._waypoints[0] - self._waypoints[n_waypoints - 1]
        self._distances[n_waypoints - 1] = numpy.sqrt(diff[0]**2 + diff[1]**2 +
                                                      diff[2]**2)

    @staticmethod
    def get_points_from_yaml_list(path):
        points = numpy.zeros([len(path), 3], dtype=numpy.float)
        for i, p in enumerate(path):
            points[i] = numpy.array([p["x"], p["y"], p["z"]], dtype=numpy.float)
        return points
