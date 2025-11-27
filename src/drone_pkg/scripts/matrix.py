#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math

class ENUtoNEDTransformer:
    def __init__(self):
        rospy.init_node('enu_to_ned_transformer')
        self.pub = rospy.Publisher('/ned_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.loginfo("ENU to NED Transformer Node Started")

    def transform_pose(self, pose):
        # Standard ENU to NED mapping:
        # NED_x (North) = ENU_y (North)
        # NED_y (East)  = ENU_x (East)
        # NED_z (Down)  = -ENU_z (Up)
        transformed_pose = PoseStamped()
        transformed_pose.header = pose.header
        transformed_pose.pose.position.x = pose.pose.position.y
        transformed_pose.pose.position.y = pose.pose.position.x
        transformed_pose.pose.position.z = -pose.pose.position.z

        # Orientation: Apply 180° rotation about X to flip Z (Up to Down)
        q_orig = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        q_rot = quaternion_from_euler(math.pi, 0, 0)  # 180° about X
        q_new = quaternion_multiply(q_rot, q_orig)
        transformed_pose.pose.orientation = Quaternion(*q_new)

        return transformed_pose

    def odom_callback(self, msg):
        try:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose

            transformed_pose = self.transform_pose(pose_msg)
            self.pub.publish(transformed_pose)

            rospy.loginfo_once("Publishing transformed NED pose with timestamp: %d.%09d",
                               transformed_pose.header.stamp.secs,
                               transformed_pose.header.stamp.nsecs)
            rospy.loginfo("NED Position: x (North): {:.3f}, y (East): {:.3f}, z (Down): {:.3f}".format(
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z
            ))
        except Exception as e:
            rospy.logerr("Error in odom_callback: %s", str(e))

if __name__ == '__main__':
    try:
        ENUtoNEDTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

