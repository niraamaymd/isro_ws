#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import math

def quaternion_multiply(q1, q2):
    # Quaternion multiplication (x, y, z, w)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return [x, y, z, w]

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [x, y, z, w]

class VisionPosePublisher:
    def __init__(self):
        rospy.init_node('vision_pose_pub')
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/rovio/odometry', Odometry, self.odom_callback)
        rospy.loginfo("Vision Pose Publisher Node Started")

    def enu_to_ned(self, pose):
        ned_pose = PoseStamped()
        ned_pose.header = pose.header

        # --- Position transformation ---
        # ENU (x: east, y: north, z: up) -> NED (x: north, y: east, z: down)
        ned_pose.pose.position.x = pose.pose.position.y
        ned_pose.pose.position.y = pose.pose.position.x
        ned_pose.pose.position.z = -pose.pose.position.z

        # --- Orientation transformation ---
        # Rotate 180 about X axis to convert ENU -> NED
        q_enu = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        q_rot = quaternion_from_euler(math.pi, 0, 0)  # 180 about X
        q_ned = quaternion_multiply(q_rot, q_enu)
        ned_pose.pose.orientation = Quaternion(*q_ned)

        return ned_pose

    def odom_callback(self, msg):
        try:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose

            # Transform ENU -> NED before publishing
            ned_pose_msg = self.enu_to_ned(pose_msg)
            self.pub.publish(ned_pose_msg)

            rospy.loginfo_once("Publishing vision pose to MAVROS in NED frame")

            rospy.loginfo_throttle(1.0, "NED Position > X: {:.3f}, Y: {:.3f}, Z: {:.3f}".format(
                ned_pose_msg.pose.position.x,
                ned_pose_msg.pose.position.y,
                ned_pose_msg.pose.position.z
            ))

        except Exception as e:
            rospy.logerr("Error in odom_callback: %s", str(e))

if __name__ == '__main__':
    try:
        VisionPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

