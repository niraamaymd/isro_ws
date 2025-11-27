#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class NEDCoordinateVerifier:
    def __init__(self):
        rospy.init_node('ned_coordinate_verifier')
        self.pub = rospy.Publisher('/ned_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.loginfo("NED Coordinate Verifier Node Started")

    def odom_callback(self, msg):
        try:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.pub.publish(pose_msg)

            # Extract position
            x = pose_msg.pose.position.x  # North
            y = pose_msg.pose.position.y  # East
            z = pose_msg.pose.position.z  # Down

            # Extract orientation as Euler angles
            q = pose_msg.pose.orientation
            quaternion = (q.x, q.y, q.z, q.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            rospy.loginfo_once("Publishing NED pose with timestamp: %d.%09d",
                               pose_msg.header.stamp.secs,
                               pose_msg.header.stamp.nsecs)
            rospy.loginfo("NED Position: X : {:.3f}, Y : {:.3f}, Z : {:.3f}".format(x, y, z))
           # rospy.loginfo("Orientation (degrees): Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(
            #    math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
           # ))
           # rospy.loginfo("Move NORTH: X should increase | Move EAST: Y should increase | Move DOWN: Z should increase")

        except Exception as e:
            rospy.logerr("Error in odom_callback: %s", str(e))

if __name__ == '__main__':
    try:
        NEDCoordinateVerifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

