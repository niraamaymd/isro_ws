#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from mavros_msgs.msg import State, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Point
from rtabmap_ros.srv import ResetPose
from tf.transformations import euler_from_quaternion
import threading

class DroneController(object):
    def __init__(self):
        rospy.init_node('drone_controller')

        self.is_connected = False
        self.is_armed = False
        self.mode = ""

        self.last_pose = None # Last healthy pose
        self.print_pose = False
        self.reset_pose_if_vision_unhealthy = True

        rospy.Subscriber("/manual_drone_cmds", String, self.manual_cmd_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb) 
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_cb)

        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        #self.set_ekf_origin()
        
        self.pause_odom() # Since in takeoff, odom is shit

        self.is_vision_pose_healthy = False


    def manual_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        rospy.loginfo("[MANUAL CMD] Received: '{}'".format(cmd))
        if cmd == "arm":
            self.arm_drone()
        elif cmd == "disarm":
            self.disarm_drone()
        elif cmd.startswith("mode"):
            parts = cmd.split()
            if len(parts) == 2:
                self.set_mode(parts[1])
            else:
                rospy.logwarn("Usage: 'mode <MODE>'")
        elif cmd in ("land", "l"):
            self.land_immediately()
        elif cmd == "takeoff":
            self.takeoff()
        elif cmd == "mission":
            threading.Thread(target=self.mission).start()
        else:
            rospy.logwarn("Unknown manual command: '{}'".format(cmd))

    def state_cb(self, msg):
        self.is_connected = msg.connected
        self.is_armed = msg.armed
        self.mode = msg.mode

        rospy.loginfo("[{} | {} | {}]".format(
            "Connected" if self.is_connected else "Disconnected",
            "Armed" if self.is_armed else "Disarmed",
            self.mode
        ))

    def pose_cb(self, msg):
        pose = msg.pose
        if pose.position.x == pose.position.y == pose.position.z == 0.0000 and \
           pose.orientation.x == pose.orientation.y == pose.orientation.z == pose.orientation.w == 0.000:
            self.is_vision_pose_healthy = False
            if self.reset_pose_if_vision_unhealthy:
                self.reset_odom_with_last_pose()
        else:
            self.is_vision_pose_healthy = True
            self.last_pose = msg.pose

        if self.print_pose:
            rospy.loginfo("Pose: {}, {}, {}".format(pose.position.x, pose.position.y, pose.position.z))

    def set_ekf_origin(self):
      	rospy.loginfo("Publishing EKF origin via topic...")

        origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', Point, queue_size=10)

        rospy.sleep(1)  # Allow time for the publisher to register with ROS master

        geo_point = Point()
        geo_point.x = 11.3218   # Latitude
        geo_point.y = 75.9361   # Longitude
        geo_point.z = 0.0       # Altitude

        if origin_pub.get_num_connections() == 0:
            rospy.logwarn("No subscribers connected to /mavros/global_position/set_gp_origin. "
                              "The message may not be received.")

        try:
            origin_pub.publish(geo_point)
            rospy.loginfo("EKF origin published: (lat: %f, lon: %f, alt: %f)", 
                              geo_point.x, geo_point.y, geo_point.z)
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish EKF origin: %s", e)


    def arm_drone(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            resp = arming_client(True)
            if resp.success:
                rospy.loginfo("Arming succeeded.")
            else:
                rospy.logerr("Arming failed. (Service call succeeded)")
        except rospy.ServiceException as e:
            rospy.logerr("Arming call failed. (err: %s)", e)

    def disarm_drone(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            resp = arming_client(False)
            if resp.success:
                rospy.loginfo("Disarming succeeded.")
            else:
                rospy.logerr("Disarming failed. (Service call succeeded)")
        except rospy.ServiceException as e:
            rospy.logerr("Disarming call failed. (err: %s)", e)
    

    def takeoff(self):
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        self.takeoff_throttle = 1650
        self.takeoff_duration = 10

        self.takeoff_start_time = rospy.get_time()
        msg.channels[2] = self.takeoff_throttle
        self.takeoff_msg = msg

        rate = rospy.Rate(10) 
        self.wait_for_pose_duration = 5
        self.wait_for_pose_start_time = None

        resetted_once_check = False

        while True:
            self.rc_override_pub.publish(self.takeoff_msg)
            if rospy.get_time() - self.takeoff_start_time >= self.takeoff_duration and not resetted_once_check:
                rospy.loginfo("TAKEOFF Completed.")
                self.reset_odom()
                self.resume_odom()
                rospy.loginfo("Waiting for vision pose")
                self.wait_for_pose_start_time = rospy.get_time()        
                resetted_once_check = True

            if self.wait_for_pose_start_time and rospy.get_time() - self.wait_for_pose_start_time >= self.wait_for_pose_duration:
                break

            rate.sleep()


    def pause_odom(self):
        try:
            rospy.wait_for_service('/rtabmap/pause_odom', timeout=1)
            pause_odom_client = rospy.ServiceProxy('/rtabmap/pause_odom', Empty)
            resp = pause_odom_client()
            rospy.loginfo("Odometry paused.")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("Odometry pause call failed. (err: %s)", e)
            
    
    def reset_odom(self):
        try:
            rospy.wait_for_service('/rtabmap/reset_odom', timeout=1)
            reset_odom_client = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
            resp = reset_odom_client()
            rospy.loginfo("Odometry resetted.")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("Odometry reset call failed. (err: %s)", e)

    def reset_odom_with_last_pose(self):
        rospy.wait_for_service('/rtabmap/reset_odom_to_pose')
        try:
            pose = self.last_pose
            orientation = pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x,
                                                     orientation.y,
                                                     orientation.z,
                                                     orientation.w])
            reset_odom = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)
            response = reset_odom(pose.position.x,
                             pose.position.y,
                             pose.position.z,
                             roll, pitch, yaw)
            rospy.loginfo("Odometry reset to pose {}".format(pose))
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)




    def resume_odom(self):
        try:
            rospy.wait_for_service('/rtabmap/resume_odom', timeout=1)
            resume_odom_client = rospy.ServiceProxy('/rtabmap/resume_odom', Empty)
            resp = resume_odom_client()
            rospy.loginfo("Odometry resumed.")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("Odometry resume call failed. (err: %s)", e)


    def hover_in_guided(self, hover_time=10):
        rospy.loginfo("STARTING MOVEMENT FOR %s secs.", hover_time)
        sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10) 
        sp_msg = PositionTarget()
        sp_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        sp_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW 

        #sp_msg.position.x = self.last_pose.position.x
        #sp_msg.position.y = self.last_pose.position.y
        #sp_msg.position.z = self.last_pose.position.z

        sp_msg.position.x = 1
        sp_msg.position.y = 0
        sp_msg.position.z = 0
        sp_msg.yaw_rate = 0
        
        hover_start_time = rospy.get_time()

        rate = rospy.Rate(10)  # 10 Hz
        while (rospy.get_time() - hover_start_time <= hover_time): 
            sp_msg.header.stamp = rospy.Time.now()
            sp_pub.publish(sp_msg)
            rate.sleep()

        rospy.loginfo("HOVER completed.")

    def hover_in_guided2(self, hover_time=10):
        rospy.loginfo("STARTING MOVEMENT FOR %s secs.", hover_time)
        sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10) 
        sp_msg = PositionTarget()
        sp_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        sp_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW 

        #sp_msg.position.x = self.last_pose.position.x
        #sp_msg.position.y = self.last_pose.position.y
        #sp_msg.position.z = self.last_pose.position.z

        sp_msg.position.x = 0 
        sp_msg.position.y = 0
        sp_msg.position.z = 0
        sp_msg.yaw_rate = 0
        
        hover_start_time = rospy.get_time()

        rate = rospy.Rate(10)  # 10 Hz
        while (rospy.get_time() - hover_start_time <= hover_time): 
            sp_msg.header.stamp = rospy.Time.now()
            sp_pub.publish(sp_msg)
            rate.sleep()

        rospy.loginfo("HOVER2 completed.")




    def mission(self):
        """
        1. Takeoff: Pause odom. Switch to FLOW_HOLD. Pilot takeoff. Reset odom. Resume odom. 
        2. Check for vision_pose estimate to come and then switch to GUIDED. 
        3. Publish current pos. continuously to hold position.
        """

        self.mission_status = "STARTED"
        rospy.loginfo("STARTED MISSION")

        self.set_mode("22") # FLOW_HOLD
        self.pause_odom()
        self.arm_drone()
        self.takeoff()

        self.stop_mission = False
        self.print_pose = True
        self.reset_pose_if_vision_unhealthy = True

        while not self.stop_mission:
            if self.is_vision_pose_healthy:
                if self.mode != "GUIDED":
                    self.set_mode("GUIDED")
                # to hold position
                self.hover_in_guided()
                self.hover_in_guided2()
                self.set_mode("LAND")
                self.stop_mission = True
            #else:
                #rospy.loginfo("VISION POSE NOT HEALTHY")
                #self.set_mode("22")
                #self.land_immediately()
                #self.stop_mission = True
        


    def set_mode(self, mode):
        rospy.wait_for_service('mavros/set_mode')
        try:
            mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
            resp = mode_client(0, mode.upper())
            if resp.mode_sent:
                rospy.loginfo("Mode set to [%s] successfully.", mode.upper())
            else:
                rospy.logerr("SetMode failed. (Service call succeeded)")

        except rospy.ServiceException as e:
            rospy.loggerr("SetMode call failed (e: %s)", e)

    def land_immediately(self):
        rospy.logwarn("LANDING NOW!")
        self.set_mode("LAND")


if __name__ == '__main__':
    node = DroneController()
    rospy.spin()
    # TODO: Add interrupts for safe landing
