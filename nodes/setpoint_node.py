#!/usr/bin/env python
from __future__ import print_function
import os
import numpy as np
import rospy
import tf.transformations
import tf2_ros
import tf2_geometry_msgs

from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
# from path_planning.path_planner import PathPlanner


class SetpointNode(Node):
    def __init__(self):
        super(SetpointNode, self).__init__("setpoint_node")

        # read waypoints
        self.waypoints = self.load_waypoints()

        # self.path_planner = PathPlanner(self.waypoints)

        self.attitude_setpoint_pub = rospy.Publisher(
            "mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        rospy.Subscriber("mavros/local_position/pose",
                         PoseStamped,
                         self.on_pose,
                         queue_size=1)

    def on_pose(self, pose_msg):
        pass

    def publish_setpoint(self):
        # get current setpoint
        thrust = 0
        roll = 0
        pitch = 0
        yaw = 0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        target = AttitudeTarget()
        target.type_mask = 0
        target.orientation.x = quat[0]
        target.orientation.y = quat[1]
        target.orientation.z = quat[2]
        target.orientation.w = quat[3]
        target.thrust = thrust
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = os.path.join(rospy.get_namespace(), "base_link") \
            .strip("/")

        self.attitude_setpoint_pub.publish(target)

    def load_waypoints(self):
        waypoints = rospy.get_param('~waypoints')
        # print(waypoints)

        # initialize waypoint matrix:
        # each row containts: number of waypoint, x, y, z, yaw
        # position in map frame
        waypoint_matrix = np.zeros([len(waypoints), 5])

        for point in waypoints:
            waypoint_matrix[point['number'], :] = np.array([point['number'],
                                                            point['x'],
                                                            point['y'],
                                                            point['z'],
                                                            point['yaw']])
        return waypoint_matrix

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self.publish_setpoint()
            rate.sleep()
        rospy.loginfo("[{}] Shutting down...".format(rospy.get_name()))


def main():
    node = SetpointNode()
    node.run()


if __name__ == '__main__':
    main()
