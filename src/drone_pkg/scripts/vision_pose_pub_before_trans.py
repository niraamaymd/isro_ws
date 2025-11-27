#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import tf.transformations as tft

class VisionPosePublisher:
    def __init__(self):
        rospy.init_node('vision_pose_pub')
        self.pub = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.loginfo("Vision Pose Publisher Node Started")

    def rotate_pose_z_90(self,pose):
        # 90 degrees around Z
        R = tft.rotation_matrix(-np.pi/2, (0, 0, 1))
        T = tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        M = np.dot(R, T)
        trans = tft.translation_from_matrix(M)
        quat = tft.quaternion_from_matrix(M)
        
        pose.position.x, pose.position.y, pose.position.z = trans
        pose.orientation = Quaternion(*quat)
        return pose


    def odom_callback(self, msg):
        try:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose
            #pose_msg.pose.pose = self.rotate_pose_z_90(pose_msg.pose.pose)
            self.pub.publish(pose_msg)

            rospy.loginfo_once("Publishing vision pose with timestamp: %d.%09d",
                               pose_msg.header.stamp.secs,
                               pose_msg.header.stamp.nsecs)

            position = pose_msg.pose.pose.position
            orientation_q = pose_msg.pose.pose.orientation
            quaternion = (
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            rospy.loginfo("x: {:.2f} y: {:.2f} z: {:.2f} roll: {:.2f} pitch: {:.2f} yaw: {:.2f}".format(
                        position.x, position.y, position.z,
                        math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

        except Exception as e:
            rospy.logerr("Error in odom_callback: %s", str(e))

if __name__ == '__main__':
    try:
        VisionPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

