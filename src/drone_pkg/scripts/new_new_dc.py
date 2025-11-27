#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Bool
from mavros_msgs.msg import State, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_srvs.srv import Empty
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_ros.srv import ResetPose
from tf.transformations import euler_from_quaternion
from my_msgs.msg import GridDetection

import threading

class DroneControllerWithSafeSpot(object):
    def __init__(self):
        rospy.init_node('drone_controller_safe_spot')

        self.is_connected = False
        self.is_armed = False
        self.mode = ""

        self.last_pose = None # Last healthy pose
        self.print_pose = False
        self.reset_pose_if_vision_unhealthy = True

        rospy.Subscriber("/manual_drone_cmds", String, self.manual_cmd_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb) 
        rospy.Subscriber("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber("/safe_spot_detected", Bool, self.safe_spot_cb)
        rospy.Subscriber("/yellow_grid", GridDetection, self.boundary_spot_cb)


        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        self.pause_odom() # Since in takeoff, odom is shit

        self.is_vision_pose_healthy = False
        self.safe_spot_detected = False
        self.last_safe_spot_time = 0.0  # Track last update time

    def manual_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        rospy.loginfo("[MANUAL CMD] Received: '{}'".format(cmd))
        if cmd == "arm":
            self.arm_drone()
        elif cmd == "disarm" or cmd == "d":
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
            threading.Thread(target=self.mission_with_safe_spot_detection).start()
        elif cmd == "set ekf":
            self.set_ekf_origin()
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
        pose = msg.pose.pose
        if pose.position.x == pose.position.y == pose.position.z == 0.0000 and \
           pose.orientation.x == pose.orientation.y == pose.orientation.z == pose.orientation.w == 0.000:
            self.is_vision_pose_healthy = False
            rospy.loginfo("VISION POSE UNHEALTHY!")
            if self.reset_pose_if_vision_unhealthy:
                self.reset_odom_with_last_pose()
        else:
            self.is_vision_pose_healthy = True
            self.last_pose = msg.pose

        if self.print_pose:
            rospy.loginfo("Pose: {}, {}, {}".format(pose.position.x, pose.position.y, pose.position.z))

    def safe_spot_cb(self, msg):
        self.safe_spot_detected = msg.data  # Directly assign the boolean value
        self.last_safe_spot_time = rospy.get_time()  # Track when we last received data
        if self.safe_spot_detected:
            rospy.loginfo("SAFE SPOT DETECTED!")
        else:
            rospy.loginfo("No safe spot detected")

    def boundary_spot_cb(self, msg):
        # Boundary is detected if any non-zero value is present in row1, row2, or row3
        self.boundary_detected = (
            any(msg.row1) or any(msg.row2) or any(msg.row3)
        )
        
        self.boundary_detect_time = rospy.get_time()  # Record the timestamp

        if self.boundary_detected:
            rospy.loginfo("BOUNDARY DETECTED!")

    
    def set_ekf_origin(self):
        rospy.loginfo("Publishing EKF origin via topic...")

        origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        rospy.sleep(1)  # Allow time for the publisher to register with ROS master

        geo_point_msg = GeoPointStamped()
        geo_point_msg.header.stamp = rospy.Time.now()
        geo_point_msg.position.latitude = 11.3218
        geo_point_msg.position.longitude = 75.9361
        geo_point_msg.position.altitude = 0.0

        if origin_pub.get_num_connections() == 0:
            rospy.logwarn("No subscribers connected to /mavros/global_position/set_gp_origin. "
                          "The message may not be received.")

        try:
            origin_pub.publish(geo_point_msg)
            rospy.loginfo("EKF origin published: (lat: %f, lon: %f, alt: %f)",
                          geo_point_msg.position.latitude,
                          geo_point_msg.position.longitude,
                          geo_point_msg.position.altitude)
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
        self.takeoff_throttle = 1670
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
                self.set_ekf_origin()
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
            pose = self.last_pose.pose
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

    def mission_with_safe_spot_detection(self):
        self.mission_status = "STARTED"
        rospy.loginfo("STARTED MISSION WITH SAFE SPOT DETECTION")

        self.set_mode("22")  # FLOW_HOLD
        self.pause_odom()
        self.arm_drone()
        self.takeoff()  # Also resumes odometry after hovering

        self.stop_mission = False
        self.reset_pose_if_vision_unhealthy = True

        # Flags to record what was detected
        safe_spot_event = False
        boundary_event = False

        while not self.stop_mission:
            if self.is_vision_pose_healthy:
                if self.mode != "LOITER":
                    self.set_mode("LOITER")

                rospy.loginfo("Applying small pitch to drive forward while monitoring safe spot and boundary")
                pitch_msg = OverrideRCIn()
                pitch_msg.channels = [0] * 18
                pitch_msg.channels[2] = 1670
                pitch_msg.channels[1] = 1480  # Small forward pitch


                pitch_start_time = rospy.get_time()
                max_pitch_duration = 30  # seconds
                rate = rospy.Rate(10)  # 10Hz

                while (rospy.get_time() - pitch_start_time < max_pitch_duration and
                    not self.safe_spot_detected and
                    not self.boundary_detected and
                    not self.stop_mission):
                    self.rc_override_pub.publish(pitch_msg)
                    rate.sleep()

                # Record the type of detection event
                if self.safe_spot_detected:
                    rospy.loginfo("Safe Spot Detected!")
                    safe_spot_event = True

                if self.boundary_detected:
                    rospy.loginfo("Yellow Boundary Detected!")
                    boundary_event = True

                # Proceed to land after detection
                if safe_spot_event:
                    self.set_mode("22")  # FLOW_HOLD
                    rospy.sleep(5)

                    rospy.loginfo("Landing due to detection")
                    self.set_mode("LAND")
                    self.pause_odom()
                    self.stop_mission = True

                elif boundary_event:
                    rospy.loginfo("Reversing direction due to boundary detection")
                    reverse_msg = OverrideRCIn()
                    reverse_msg.channels = [0] * 18
                    reverse_msg.channels[2] = 1650
                    reverse_msg.channels[1] = 1520  # Small backward pitch

                    reverse_start_time = rospy.get_time()
                    reverse_duration = 3  # Move backward for 3 seconds

                    while rospy.get_time() - reverse_start_time < reverse_duration and not self.stop_mission:
                        self.rc_override_pub.publish(reverse_msg)
                        rate.sleep()

                        

                else:
                    rospy.loginfo("No safe spot or boundary detected within time limit, landing")
                    self.set_mode("LAND")
                    self.pause_odom()
                    self.stop_mission = True




            else:
                rospy.loginfo("VISION POSE NOT HEALTHY, landing")
                self.set_mode("LAND")
                self.pause_odom()
                self.stop_mission = True

        # After mission ends, log the cause
        if safe_spot_event:
            rospy.loginfo("Mission ended due to SAFE SPOT detection.")

        if boundary_event:
            rospy.loginfo("Mission ended due to YELLOW BOUNDARY detection.")

        if not safe_spot_event and not boundary_event:
            rospy.loginfo("Mission ended without any detection event.")

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
    node = DroneControllerWithSafeSpot()
    rospy.spin()
    # TODO: Add interrupts for safe landing

