#!/usr/bin/env python
import os
import numpy as np
import rospy
import tf.transformations
import tf2_ros
import tf2_geometry_msgs

from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget


class SetpointNode(Node):
    def __init__(self):
        super(SetpointNode, self).__init__("setpoint_node")

        self.attitude_setpoint_pub = rospy.Publisher(
            "mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        
        rospy.Subscriber("mavros/local_position/pose",
                         PoseStamped,
                         self.on_pose,
                         queue_size=1)

    def on_pose(self):
        pass

    def _publish_setpoint(self):
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

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self._publish_setpoint()
            rate.sleep()
        rospy.loginfo("[{}] Shutting down...".format(rospy.get_name()))


def main():
    node = SetpointNode()
    node.run()


if __name__ == '__main__':
    main()
