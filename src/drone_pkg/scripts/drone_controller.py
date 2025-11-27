#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest, CommandLongRequest
from rtabmap_ros.srv import ResetPose
from collections import deque
import math
import time
from tf.transformations import euler_from_quaternion

class DroneController(object):
    def __init__(self):
        rospy.init_node('drone_controller')

        # STATE VARIABLES
        self.connected = False
        self.armed = False
        self.m1_started = False
        self.m2_resume = False
        self.remaining_throttle_time = 0.0
        self.remaining_forward_time = 0.0
        self.flowhold_pause = False
        self.mode = ""

        # SLAM tracking and buffer (unused for mission2 but retained for future VIO)
        self.slam_tracking_state = False
        self.pose_buffer = deque(maxlen=10)

        self.current_pose = None
        self.last_pose = None 
        self.tracking_state = False
        self.mode_before_non_tracking = None

        # Pixhawk local position (x, y, z in NED)
        self.current_local_pose = None

        # MISSION FRAMEWORK
        self.mission_active = False
        self.mission_id = None # e.g. "mission1", "mission2", "mission3"
        self.mission_step = 0
        self.mission_timer = None # for periodic tasks (e.g., throttle override)
        self.landing_delay_timer = None
        self.throttle_counter = 0

        # Mission2 timing checkpoints & flags
        self.m2_takeoff_time = None
        self.m2_forward_start = None
        self.m2_side_start = None
        self.m2_reverse_start = None
        self.m2_left_start = None
        self.m2_hover_start = None
        self.m2_forward_sent = False
        self.m2_side_sent = False
        self.m2_reverse_sent = False
        self.m2_left_sent = False

        # Mission2 parameters
        self.M2_TAKEOFF_ALT = 2.0 # meters
        self.M2_FORWARD_DIST = 2.0 # CHANGED TO 2 METERS (body-forward)
        self.M2_SIDE_DIST = 1.0 # meters (body-right)
        self.M2_HOVER_AFTER = 10.0 # seconds before landing

        # Mission3 timing checkpoints & flags
        self.m3_takeoff_time = None
        self.m3_forward_sent = False
        self.m3_side_sent = False
        self.m3_reverse_sent = False
        self.m3_left_sent = False

        # Mission3 parameters (reuse the same distances/altitude as Mission2)
        self.M3_TAKEOFF_ALT = self.M2_TAKEOFF_ALT
        self.M3_FORWARD_DIST = self.M2_FORWARD_DIST
        self.M3_SIDE_DIST = self.M2_SIDE_DIST
        self.M3_HOVER_AFTER = 0.0 # not used in Mission 3 (we land immediately)
        self.M3_LAND_WAIT = 10.0 # seconds to wait on ground between moves

        # PUBLISHERS & SUBSCRIBERS
        rospy.Subscriber("/manual_drone_cmds", String, self.manual_drone_cmd_cb, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        rospy.Subscriber("/orbslam_pose", PoseStamped, self.slam_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_position_cb, queue_size=10)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_cb)

        self.vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.body_setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(0.1), lambda evt: self.check_mission_progression())
        rospy.Timer(rospy.Duration(0.1), lambda evt: self.vision_pose_cb())
        self.dummy_timer = rospy.Timer(rospy.Duration(0.2), lambda evt: self.publish_dummy_setpoint())

        rospy.loginfo("DroneController initialized.")

    # CALLBACKS FOR MANUAL COMMANDS

    def manual_drone_cmd_cb(self, msg):
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
            self._land_immediately()
        elif cmd.startswith("start mission"):
            parts = cmd.split()
            if len(parts) == 2:
                self.start_mission(parts[1])
            else:
                rospy.logwarn("Usage: 'start missionX' (e.g. mission1, mission2, mission3)")
        else:
            rospy.logwarn("Unknown manual command: '{}'".format(cmd))

    def start_mission(self, mission_id):
        if self.mission_active:
            rospy.logwarn("[MISSION] A mission is already running.")
            return
        rospy.loginfo("[MISSION] Starting {}...".format(mission_id))
        self.mission_active = True
        self.mission_id = mission_id
        self.mission_step = 1
        if mission_id == "mission1":
            self.set_mode("STABILIZE")
        elif mission_id == "mission2":
            self.mission_active = True
            self.mission_id = "mission2"
            self.mission_step = 1
            if self.m1_started:
                rospy.loginfo("[MISSION2-RESUME] Mission1 done → resuming in-air logic")
                self.set_mode("GUIDED")  # CHANGED FROM LOITER TO GUIDED
                self.m2_resume = True
            else:
                rospy.loginfo("[MISSION2] Starting fresh GUIDED takeoff")
                self.set_mode("GUIDED")
                self.m2_resume = False
        elif mission_id == "mission3":
            self.set_mode("GUIDED")
        else:
            rospy.logerr("[MISSION] Unknown mission: {}".format(mission_id))
            self.mission_active = False
            self.mission_id = None
            self.mission_step = 0

    # MAVROS STATE CALLBACK

    def state_cb(self, msg):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode
        print("STATE: [ %s / %s /, %s ]" % (
            "Connected" if self.connected else "Not connected",
            "Armed" if self.armed else "Disarmed",
            self.mode
        ))

    # CHECKS VISION POSE PUBLISHED FROM SLAM

    def pose_cb(self, msg):
      self.current_pose = msg.pose

      # Define what constitutes a "bad" pose (tracking lost)
      is_bad_pose = (msg.pose.orientation.w == 0.000 and msg.pose.position.x == 0.000 and msg.pose.position.z == 0.000)

      # 1. If tracking is lost, switch to FLOW_HOLD and reset odometry
      if is_bad_pose:
          if not self.tracking_state:  # Already in tracking lost state, do nothing
              return
          rospy.logwarn("[ERROR] BAD VISUAL ODOM - Switching to FLOW_HOLD")
          if self.mode != "CMODE(22)":
            self.mode_before_non_tracking = self.mode
            rospy.loginfo("SET MODE BEFORE NON_TRACKING AS {}".format(self.mode))
            self.set_mode("22")  # FLOW_HOLD
          if self.last_pose:
              self.reset_odom_by_pose(self.last_pose)
          self.tracking_state = False

      # 2. If tracking is recovered while in FLOW_HOLD, restore the previous mode
      else:
          if not self.tracking_state:
              if self.mode_before_non_tracking and self.mode == "CMODE(22)":
                  rospy.loginfo("[RECOVERY] Good pose detected - Restoring previous mode: %s", self.mode_before_non_tracking)
                  self.set_mode(self.mode_before_non_tracking)
                  self.mode_before_non_tracking = None
          self.last_pose = self.current_pose
          self.tracking_state = True

    def reset_odom_by_pose(self,pose):
      rospy.wait_for_service('/rtabmap/reset_odom_to_pose')
      try:
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

    # LOCAL POSITION CALLBACK (PIXHAWK)

    def local_position_cb(self, msg):
        self.current_local_pose = msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        if self.flowhold_pause and not (x == 0.0 and y == 0.0):
            rospy.loginfo("[M2-RESUME] Nav recovered → resuming mission2")
            self.flowhold_pause = False
            if hasattr(self, "remaining_throttle_time") and self.remaining_throttle_time > 0:
                self.resume_start_time = time.time() - (5.0 - self.remaining_throttle_time)
                self.mission_timer = rospy.Timer(rospy.Duration(0.1), self._publish_throttle_only)
                self.mission_step = 2
            elif hasattr(self, "remaining_forward_time") and self.remaining_forward_time > 0:
                self.move_start_time = time.time() - (5.0 - self.remaining_forward_time)
                self.mission_timer = rospy.Timer(rospy.Duration(0.1), self._publish_throttle_and_pitch)
                self.mission_step = 3

    # SLAM CALLBACK & VISION-POSE BROADCAST

    def slam_cb(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        if all([pos.x == 0.0, pos.y == 0.0, pos.z == 0.0]) and ori.w == 1.0:
            if self.slam_tracking_state:
                rospy.logwarn("[SLAM] ORB-SLAM STOPPED TRACKING!")
            self.slam_tracking_state = False
            return
        self.slam_tracking_state = True
        self.pose_buffer.append(msg)

    def vision_pose_cb(self):
        if self.pose_buffer:
            pose = self.pose_buffer.popleft()
            self.vision_pose_pub.publish(pose)
            rospy.logdebug("[VISION] Published vision pose (stamp: {})".format(pose.header.stamp))

    # ARM / DISARM / SET_MODE

    def arm_drone(self):
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            resp = arming_client(True)
            if resp.success:
                rospy.loginfo("[SERVICE] Arming succeeded")
            else:
                rospy.logerr("[SERVICE] Arming failed! Result: %s", resp.result)
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] Arming call failed: %s", e)

    def disarm_drone(self):
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            resp = arming_client(False)
            if resp.success:
                rospy.loginfo("[SERVICE] Disarming succeeded")
            else:
                rospy.logerr("[SERVICE] Disarming failed! Result: %s", resp.result)
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] Disarming call failed: %s", e)

    def set_mode(self, mode):
        mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/set_mode')
        try:
            resp = mode_client(0, mode.upper())
            if resp.mode_sent:
                rospy.loginfo("[SERVICE] SetMode(%s) succeeded", mode.upper())
            else:
                rospy.logerr("[SERVICE] SetMode(%s) failed", mode.upper())
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] SetMode call failed: %s", e)

    def _land_immediately(self):
        rospy.logwarn("[SAFETY] Forcing LAND mode now!")
        self.set_mode("LAND")

    # DUMMY SETPOINT (TO SATISFY PX4 OFFBOARD REQUIREMENT)

    def publish_dummy_setpoint(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.body_setpoint_pub.publish(msg)
        if self.mission_active and self.mission_id in ("mission2", "mission3") and self.mode.upper() in ("GUIDED", "OFFBOARD"):
            self.dummy_timer.shutdown()

    # PUBLISH BODY-FRAME OFFSET

    def publish_body_offset(self, forward, right):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        msg.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        msg.position.x = forward
        msg.position.y = right
        msg.position.z = 0.0
        msg.yaw_rate = 0.0
        self.body_setpoint_pub.publish(msg)

    # MISSION PROGRESSION

    def check_mission_progression(self, evt=None):
        if self.mission_active and self.mission_id is not None:
            if self.mission_id == "mission1":
                self.execute_mission1_step()
            elif self.mission_id == "mission2":
                self.execute_mission2_step()
            elif self.mission_id == "mission3":
                self.execute_mission3_step()

    # MISSION1: ORIGINAL SEQUENCE

    def execute_mission1_step(self):
        if self.mission_step == 1 and self.mode.upper() == "STABILIZE":
            self.m1_started = True
            rospy.loginfo("[MISSION1] STABILIZE confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2
        elif self.mission_step == 2 and self.armed:
            rospy.loginfo("[MISSION1] Armed → Switching to FLOW_HOLD...")
            self.set_mode("22")
            self.mission_step = 3
        elif self.mission_step == 3 and self.mode.upper() == "CMODE(22)":
            rospy.loginfo("[MISSION1] Entered FLOW_HOLD → Throttle override for 10 s...")
            self.throttle_counter = 0
            self.mission_timer = rospy.Timer(rospy.Duration(0.1), lambda evt: self.publish_throttle())
            self.mission_step = 4

    def publish_throttle(self):
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        msg.channels[2] = 1680
        self.rc_override_pub.publish(msg)
        self.throttle_counter += 1
        rospy.loginfo(self.throttle_counter)
        if self.throttle_counter >= 100:
            rospy.loginfo("[MISSION1] 10 s override complete → Switching to LAND")
            if self.mission_timer:
                self.mission_timer.shutdown()
                self.mission_timer = None
            #self._land_immediately()
            #self.landing_delay_timer = rospy.Timer(rospy.Duration(15.0), lambda evt: self.disarm_after_landing(), oneshot=True)
            #self.mission_active = False
            #self.mission_id = None
            self.mission_id = "mission2"
            self.mission_step = 1
            self.m2_resume = True
            self.set_mode("GUIDED")  # CHANGED FROM LOITER TO GUIDED

    def disarm_after_landing(self):
        rospy.loginfo("[MISSION1] 15 s LAND delay elapsed → Disarming now")
        self.disarm_drone()
        if self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None

    # MISSION2: GUIDED TAKEOFF & BODY-FRAME MOVES

    def execute_mission2_step(self):
        if self.m2_resume:
            return self._execute_m2_resume()
        if self.mission_step == 1 and self.mode.upper() == "GUIDED":
            rospy.loginfo("[MISSION2] GUIDED confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2
        elif self.mission_step == 2 and self.armed:
            command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            rospy.wait_for_service('/mavros/cmd/command')
            try:
                resp = command_client(
                    0, 0, 22, 0, 0, 0, 0, 0, 0, float(self.M2_TAKEOFF_ALT)
                )
                if resp.success:
                    rospy.loginfo("[MISSION2] Takeoff to {} m requested.".format(self.M2_TAKEOFF_ALT))
                else:
                    rospy.logerr("[MISSION2] Takeoff command failed")
            except rospy.ServiceException as e:
                rospy.logerr("[MISSION2] Service call failed: %s", e)
                return
            self.m2_takeoff_time = time.time()
            self.mission_step = 3
        elif self.mission_step == 3:
            if self.m2_takeoff_time and time.time() - self.m2_takeoff_time >= 10.0:
                if not self.m2_forward_sent:
                    rospy.loginfo("[MISSION2] Entering FORWARD phase → sending 2 m forward (body-frame)…")
                    self.publish_body_offset(self.M2_FORWARD_DIST, 0.0)
                    self.m2_forward_sent = True
                if time.time() - self.m2_takeoff_time >= 20.0:
                    rospy.loginfo("[MISSION2] FORWARD phase COMPLETE → proceeding to SIDE phase.")
                    self.m2_side_start = time.time()
                    self.mission_step = 4
        elif self.mission_step == 4:
            elapsed = time.time() - self.m2_side_start
            if not self.m2_side_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering SIDE phase → sending 1 m right (body-frame)…")
                self.publish_body_offset(0.0, self.M2_SIDE_DIST)
                self.m2_side_sent = True
            if elapsed >= 10.0:
                rospy.loginfo("[MISSION2] SIDE phase COMPLETE → proceeding to REVERSE phase.")
                self.m2_reverse_start = time.time()
                self.mission_step = 5
        elif self.mission_step == 5:
            elapsed = time.time() - self.m2_reverse_start
            if not self.m2_reverse_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering REVERSE phase → sending 2 m reverse (body-frame)…")
                self.publish_body_offset(-self.M2_FORWARD_DIST, 0.0)
                self.m2_reverse_sent = True
            if elapsed >= 10.0:
                rospy.loginfo("[MISSION2] REVERSE phase COMPLETE → proceeding to LEFT phase.")
                self.m2_left_start = time.time()
                self.mission_step = 6
        elif self.mission_step == 6:
            elapsed = time.time() - self.m2_left_start
            if not self.m2_left_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering LEFT phase → sending 1 m left (body-frame)…")
                self.publish_body_offset(0.0, -self.M2_SIDE_DIST)
                self.m2_left_sent = True
            if elapsed >= 10.0:
                rospy.loginfo("[MISSION2] LEFT phase COMPLETE → hovering for {} s…".format(self.M2_HOVER_AFTER))
                self.m2_hover_start = time.time()
                self.mission_step = 7
        elif self.mission_step == 7:
            elapsed = time.time() - self.m2_hover_start
            if elapsed >= self.M2_HOVER_AFTER:
                rospy.loginfo("[MISSION2] Hover complete → Switching to LAND")
                self._land_immediately()
                rospy.Timer(rospy.Duration(15.0), lambda evt: self.disarm_after_landing_m2(), oneshot=True)
                self.mission_active = False
                self.mission_step = 0

    def _execute_m2_resume(self):
        if self.mission_step == 1:
            rospy.loginfo("[M2-RESUME] GUIDED confirmed → Moving 2m forward (body-frame)")
            # Directly publish body offset in GUIDED mode
            self.publish_body_offset(2.0, 0.0)
            self.move_start_time = time.time()
            self.mission_step = 2
        elif self.mission_step == 2:
            if time.time() - self.move_start_time >= 5.0:  # Wait 5 seconds for move
                rospy.loginfo("[M2-RESUME] Forward move complete → LAND")
                self._land_immediately()
                rospy.Timer(rospy.Duration(15.0), lambda evt: self.disarm_after_landing_m2(), oneshot=True)
                self.mission_active = False
                self.mission_step = 0

    # Removed throttle-only and throttle+pitch methods for resume path
    # since we're using guided mode body offsets instead

    def disarm_after_landing_m2(self):
        rospy.loginfo("[MISSION2] Disarming after LAND.")
        self.disarm_drone()

    # MISSION3: GUIDED TAKEOFF & BODY-FRAME MOVES WITH LAND/TAKEOFF BETWEEN EACH

    def execute_mission3_step(self):
        if self.mission_step == 1 and self.mode.upper() == "GUIDED":
            rospy.loginfo("[MISSION3] GUIDED confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2
        elif self.mission_step == 2 and self.armed:
            command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            rospy.wait_for_service('/mavros/cmd/command')
            try:
                resp = command_client(
                    0, 0, 22, 0, 0, 0, 0, 0, 0, float(self.M3_TAKEOFF_ALT)
                )
                if resp.success:
                    rospy.loginfo("[MISSION3] Takeoff to {} m requested.".format(self.M3_TAKEOFF_ALT))
                else:
                    rospy.logerr("[MISSION3] Takeoff command failed")
            except rospy.ServiceException as e:
                rospy.logerr("[MISSION3] Service call failed: %s", e)
                return
            self.m3_takeoff_time = time.time()
            self.mission_step = 3
        elif self.mission_step == 3:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_forward_sent:
                rospy.loginfo("[MISSION3] Entering FORWARD phase → sending 2 m forward (body-frame)…")
                self.publish_body_offset(self.M3_FORWARD_DIST, 0.0)
                self.m3_forward_sent = True
                rospy.loginfo("[MISSION3] FORWARD offset sent → Switching to LAND")
                self._land_immediately()
                self.landing_delay_timer = rospy.Timer(rospy.Duration(self.M3_LAND_WAIT), lambda evt: self.m3_takeoff_for_side(), oneshot=True)
        elif self.mission_step == 5:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_side_sent:
                rospy.loginfo("[MISSION3] Entering SIDE phase → sending 1 m right (body-frame)…")
                self.publish_body_offset(0.0, self.M3_SIDE_DIST)
                self.m3_side_sent = True
                rospy.loginfo("[MISSION3] SIDE offset sent → Switching to LAND")
                self._land_immediately()
                self.landing_delay_timer = rospy.Timer(rospy.Duration(self.M3_LAND_WAIT), lambda evt: self.m3_takeoff_for_reverse(), oneshot=True)
        elif self.mission_step == 7:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_reverse_sent:
                rospy.loginfo("[MISSION3] Entering REVERSE phase → sending 2 m reverse (body-frame)…")
                self.publish_body_offset(-self.M3_FORWARD_DIST, 0.0)
                self.m3_reverse_sent = True
                rospy.loginfo("[MISSION3] REVERSE offset sent → Switching to LAND")
                self._land_immediately()
                self.landing_delay_timer = rospy.Timer(rospy.Duration(self.M3_LAND_WAIT), lambda evt: self.m3_takeoff_for_left(), oneshot=True)
        elif self.mission_step == 9:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_left_sent:
                rospy.loginfo("[MISSION3] Entering LEFT phase → sending 1 m left (body-frame)…")
                self.publish_body_offset(0.0, -self.M3_SIDE_DIST)
                self.m3_left_sent = True
                rospy.loginfo("[MISSION3] LEFT offset sent → Switching to LAND")
                self._land_immediately()
                self.landing_delay_timer = rospy.Timer(rospy.Duration(15.0), lambda evt: self.disarm_after_landing_m3(), oneshot=True)
                self.mission_active = False
                self.mission_id = None
                self.mission_step = 0

    def m3_takeoff_for_side(self):
        if self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None
        command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            resp = command_client(
                0, 0, 22, 0, 0, 0, 0, 0, 0, float(self.M3_TAKEOFF_ALT)
            )
            if resp.success:
                rospy.loginfo("[MISSION3] Re-Takeoff to {} m for SIDE phase requested.".format(self.M3_TAKEOFF_ALT))
            else:
                rospy.logerr("[MISSION3] Takeoff command failed")
        except rospy.ServiceException as e:
            rospy.logerr("[MISSION3] Takeoff service call failed: %s", e)
        self.m3_takeoff_time = time.time()
        self.mission_step = 5

    def m3_takeoff_for_reverse(self):
        if self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None
        command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            resp = command_client(
                0, 0, 22, 0, 0, 0, 0, 0, 0, float(self.M3_TAKEOFF_ALT)
            )
            if resp.success:
                rospy.loginfo("[MISSION3] Re-Takeoff to {} m for REVERSE phase requested.".format(self.M3_TAKEOFF_ALT))
            else:
                rospy.logerr("[MISSION3] Takeoff command failed")
        except rospy.ServiceException as e:
            rospy.logerr("[MISSION3] Takeoff service call failed: %s", e)
        self.m3_takeoff_time = time.time()
        self.mission_step = 7

    def m3_takeoff_for_left(self):
        if self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None
        command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            resp = command_client(
                0, 0, 22, 0, 0, 0, 0, 0, 0, float(self.M3_TAKEOFF_ALT)
            )
            if resp.success:
                rospy.loginfo("[MISSION3] Re-Takeoff to {} m for LEFT phase requested.".format(self.M3_TAKEOFF_ALT))
            else:
                rospy.logerr("[MISSION3] Takeoff command failed")
        except rospy.ServiceException as e:
            rospy.logerr("[MISSION3] Takeoff service call failed: %s", e)
        self.m3_takeoff_time = time.time()
        self.mission_step = 9

    def disarm_after_landing_m3(self):
        rospy.loginfo("[MISSION3] Disarming after final LAND.")
        self.disarm_drone()
        if hasattr(self, 'landing_delay_timer') and self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None

    # SHUTDOWN / SAFETY LAND HANDLER

    def safety_land_and_shutdown(self):
        if self.mode.upper() != "LAND":
            rospy.logwarn("[SAFETY] Node shutting down unexpectedly → Forcing LAND")
            self._land_immediately()
            rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('drone_controller')
    drone_controller = DroneController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[MAIN] KeyboardInterrupt detected → Safety landing")
        try:
            drone_controller.safety_land_and_shutdown()
        except Exception as e:
            rospy.logerr("[MAIN] Exception during safety landing: {}".format(e))
    except Exception as e1:
        rospy.logerr("[MAIN] Unexpected exception: {} → Safety landing".format(e1))
        try:
            drone_controller.safety_land_and_shutdown()
        except Exception as e2:
            rospy.logerr("[MAIN] Exception during exception-handler landing: {}".format(e2))
    finally:
        rospy.loginfo("[MAIN] shutting down node")

