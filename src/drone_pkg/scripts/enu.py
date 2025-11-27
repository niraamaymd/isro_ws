#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class ENUPosePublisher:
    def __init__(self):
        rospy.init_node('enu_pose_pub')
        self.pub = rospy.Publisher('/enu_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.loginfo("ENU Pose Publisher Node Started")

    def odom_callback(self, msg):
        try:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.pub.publish(pose_msg)
            rospy.loginfo_once("Publishing ENU pose with timestamp: %d.%09d",
                               pose_msg.header.stamp.secs,
                               pose_msg.header.stamp.nsecs)
            rospy.loginfo("ENU X: {:.3f}, Y: {:.3f}, Z: {:.3f}".format(
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z
            ))
        except Exception as e:
            rospy.logerr("Error in odom_callback: %s", str(e))

if __name__ == '__main__':
    try:
        ENUPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

