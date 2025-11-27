#!/usr/bin/env python
# coding: utf-8
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from mavros_msgs.msg import State, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandLong

from collections import deque
import math
import time


class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        # super().__init__('drone_controller')

        #  STATE VARIABLES 
        self.connected = False
        self.armed = False
        self.m1_started = False
        self.m2_resume = False
        self.remaining_throttle_time = 0.0
        self.remaining_forward_time  = 0.0
        self.flowhold_pause   = False
        self.mode = ""

        # Futures for async service calls
        self.arm_future = None
        self.disarm_future = None
        self.set_mode_future = None

        # SLAM tracking and buffer (unused for mission2 but retained for future VIO)
        self.slam_tracking_state = False
        self.pose_buffer = deque(maxlen=10)

        # Pixhawk local position (x, y, z in NED)
        self.current_local_pose = None

        #  MISSION FRAMEWORK 
        self.mission_active = False
        self.mission_id = None        # e.g. "mission1", "mission2", "mission3"
        self.mission_step = 0
        self.mission_timer = None     # for periodic tasks (e.g., throttle override)
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
        self.M2_TAKEOFF_ALT = 2.0      # meters
        self.M2_FORWARD_DIST = 3.0     # meters (body-forward)
        self.M2_SIDE_DIST = 1.0        # meters (body-right)
        self.M2_HOVER_AFTER = 10.0     # seconds before landing

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
        self.M3_HOVER_AFTER = 0.0      # not used in Mission 3 (we land immediately)
        self.M3_LAND_WAIT = 10.0       # seconds to wait on ground between moves

        #  PUBLISHERS & SUBSCRIBERS 
        # 1) Manual commands (arm/disarm/mode/start missionX)
        rospy.Subscriber("/manual_drone_cmds", String, self.manual_drone_cmd_cb, queue_size=10)

        # 2) MAVROS state updates
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)

        # 3) ORB-SLAM pose (fallback for vision); not used in mission2/3
        rospy.Subscriber("/orbslam_pose", PoseStamped, self.slam_cb, queue_size=10)

        # 4) Pixhawk local position (to check altitude if needed)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_position_cb, queue_size=10)

        self.vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        
        # 6) RC override (used in mission1)
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        # 7) Raw setpoint publisher for body‐frame offsets (mission2 & mission3)
        self.body_setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) 
        # self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)          
        # self.command_long_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        #  TIMERS 
        # 1) Check service-call futures & mission progression at 10 Hz
        rospy.Timer(rospy.Duration(0.1), lambda evt: self.check_futures())

        # 2) Publish buffered SLAM poses at 10 Hz
        rospy.Timer(rospy.Duration(0.1), lambda evt: self.vision_pose_cb())

        # Until we enter GUIDED/OFFBOARD, we need to continuously stream something
        # to keep the autopilot happy. We'll stream a dummy setpoint at 5 Hz.
        self.dummy_timer = rospy.Timer(rospy.Duration(0.2), lambda evt: self.publish_dummy_setpoint())

        rospy.loginfo("DroneController initialized.")

    #
    #  CALLBACKS FOR MANUAL COMMANDS 
    #
    def manual_drone_cmd_cb(self, msg):
        """
        Accepts:
          - "arm"
          - "disarm"
          - "mode <MODE_NAME>"
          - "land" or "l"
          - "start mission1"
          - "start mission2"
          - "start mission3"
        """
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
                rospy.logwarn("Usage: 'mode <MODE_NAME>'")
        elif cmd in ("land", "l"):
            self._land_immediately()
        elif cmd.startswith("start mission"):
            parts = cmd.split()
            if len(parts) == 2:
                self.start_mission(parts[1])  # mission1 or mission2 or mission3
            else:
                rospy.logwarn("Usage: 'start missionX' (e.g. mission1, mission2, mission3)")
        else:
            rospy.logwarn("Unknown manual command: '{}'".format(cmd))

    def start_mission(self, mission_id):
        """
        Kicks off an autonomous mission:
          - mission1: original (STABILIZE → arm → FLOW_HOLD override → LAND → disarm)
          - mission2: (GUIDED → arm → takeoff to 2 m → forward → right → reverse → left → hover → LAND → disarm)
          - mission3: (GUIDED → arm → takeoff to 2 m → forward → LAND → wait 10 s → takeoff → right → LAND → wait 10 s → takeoff → reverse → LAND → wait 10 s → takeoff → left → LAND → disarm)
        """
        if self.mission_active:
            rospy.logwarn("[MISSION] A mission is already running.")
            return

        rospy.loginfo("[MISSION] Starting {}...".format(mission_id))
        self.mission_active = True
        self.mission_id = mission_id
        self.mission_step = 1

        if mission_id == "mission1":
            # Step 1 → switch to STABILIZE
            self.set_mode("STABILIZE")

        elif mission_id == "mission2":
            self.mission_active = True
            self.mission_id = "mission2"
            self.mission_step = 1

            if self.m1_started:
                rospy.loginfo("[MISSION2‑RESUME] Mission1 done → resuming in‑air logic")
                # go straight to LOITRE
                self.set_mode("LOITRE")
                self.m2_resume = True
            else:
                rospy.loginfo("[MISSION2] Starting fresh GUIDED takeoff")
                self.set_mode("GUIDED")
                self.m2_resume = False

        elif mission_id == "mission3":
            # Step 1 → switch to GUIDED
            self.set_mode("GUIDED")

        else:
            rospy.logerr("[MISSION] Unknown mission: {}".format(mission_id))
            self.mission_active = False
            self.mission_id = None
            self.mission_step = 0

    #
    #  MAVROS STATE CALLBACK 
    #
    def state_cb(self, msg):
        """
        Update connected, armed status, and current mode.
        """
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        print("STATE: [ %s / %s /, %s ]" % (
            "Connected" if self.connected else "Not connected",
            "Armed"     if self.armed    else "Disarmed",
            self.mode
        ))

    #
    #  LOCAL POSITION CALLBACK (PIXHAWK) 
    #
    def local_position_cb(self, msg):
        """
        Receive /mavros/local_position/pose from Pixhawk/EKF.
        """
        self.current_local_pose = msg
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 2) If we had paused in FLOW_HOLD because nav was (0,0),
        #    and now x,y are non‑zero → resume the RC‑override timers
        if self.flowhold_pause and not (x == 0.0 and y == 0.0):
            rospy.loginfo("[M2‑RESUME] Nav recovered → resuming mission2")
            self.flowhold_pause = False

            # Resume whichever step we were in:
            if hasattr(self, "remaining_throttle_time") and self.remaining_throttle_time > 0:
                # restart throttle‑only timer with adjusted start time
                self.resume_start_time = time.time() - (5.0 - self.remaining_throttle_time)
                self.mission_timer = self.create_timer(0.1, self._publish_throttle_only)
                self.mission_step = 2

            elif hasattr(self, "remaining_forward_time") and self.remaining_forward_time > 0:
                # restart throttle+ pitch timer
                self.move_start_time = time.time() - (5.0 - self.remaining_forward_time)
                self.mission_timer = self.create_timer(0.1, self._publish_throttle_and_pitch)
                self.mission_step = 3

    #
    #  SLAM CALLBACK & VISION-POSE BROADCAST 
    #
    def slam_cb(self, msg):
        """
        Buffer valid SLAM poses (for future VIO use).
        """
        pos = msg.pose.position
        ori = msg.pose.orientation

        # Heuristic: if pose is all zeros with w=1, SLAM lost tracking
        if all([pos.x == 0.0, pos.y == 0.0, pos.z == 0.0]) and ori.w == 1.0:
            if self.slam_tracking_state:
                rospy.logwarn("[SLAM] ORB-SLAM STOPPED TRACKING!")
                self.slam_tracking_state = False
            return

        self.slam_tracking_state = True
        self.pose_buffer.append(msg)

    def vision_pose_cb(self):
        """
        Publish oldest buffered SLAM pose (if any) to /mavros/vision_pose/pose at 10 Hz.
        """
        if self.pose_buffer:
            pose = self.pose_buffer.popleft()
            self.vision_pose_pub.publish(pose)
            # Debug-level log to avoid spam
            rospy.logdebug("[VISION] Published vision pose (stamp: {})".format(pose.header.stamp))


    #
    #  ARM / DISARM / SET_MODE 
    #
    def arm_drone(self):
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) 
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
          arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
          req = CommandBool.Request()
          req.value = True
          self.arm_future = arming_client.call_async(req)
          rospy.loginfo("[SERVICE] Arming requested...")
          return
        except rospy.ServiceException as e:
          print("Arming Service call failed: %s"%e)


    def disarm_drone(self):
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) 
        if not self.arming_client.wait_for_service(1.0):
            rospy.logerr("Disarming service not available!")
            return
        req = CommandBool.Request()
        req.value = False
        self.disarm_future = self.arming_client.call_async(req)
        rospy.loginfo("[SERVICE] Disarming requested...")

    def set_mode(self, mode):
        if not self.mode_client.wait_for_service(1.0):
            rospy.logerr("SetMode service not available!")
            return
        req = SetMode.Request()
        req.custom_mode = mode.upper()
        self.set_mode_future = self.mode_client.call_async(req)
        rospy.loginfo("[SERVICE] SetMode({}) requested...".format(mode.upper()))

    def _land_immediately(self):
        rospy.logwarn("[SAFETY] Forcing LAND mode now!")
        self.set_mode("LAND")

    #
    #  DUMMY SETPOINT (TO SATISFY PX4 OFFBOARD REQUIREMENT) 
    #
    def publish_dummy_setpoint(self):
        """
        Publish a benign PositionTarget at 5 Hz until we switch to GUIDED/OFFBOARD.
        Keeps PX4’s offboard tracker alive.
        """
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()

        # Ignore all fields except timestamp
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
        # Use any frame since no fields matter
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.body_setpoint_pub.publish(msg)

        # Once we enter GUIDED or OFFBOARD, cancel this dummy timer
        if self.mission_active and self.mission_id in ("mission2", "mission3") and self.mode.upper() in ("GUIDED", "OFFBOARD"):
            self.dummy_timer.cancel()

    #
    #  PUBLISH BODY-FRAME OFFSET 
    #
    def publish_body_offset(self, forward, right):
        """
        Publish a PositionTarget in FRAME_BODY_NED:
          - position.x = forward (body forward)
          - position.y = right   (body right)
          - position.z = 0.0     (no vertical offset)
        """
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()


        # We only provide Px, Py (body‐frame). Ignore velocities/accels/yaw/yaw_rate.
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )

        # FRAME_BODY_OFFSET_NED = 8
        msg.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        # In body NED: x=forward, y=right, z=down (positive). z=0 means hold current altitude.
        msg.position.x = forward
        msg.position.y = right
        msg.position.z = 0.0
        msg.yaw_rate = 0.0  # No yaw rate

        self.body_setpoint_pub.publish(msg)

    #
    #  MISSION PROGRESSION & FUTURE-CHECK LOGIC 
    #
    def check_futures(self):
        """
        Runs at 10 Hz:
          1) Advance mission logic based on mission_id & mission_step.
          2) Check any pending service-call futures (arm, disarm, set_mode) for completion.
        """
        #  (1) Mission progression 
        if self.mission_active and self.mission_id is not None:
            if self.mission_id == "mission1":
                self.execute_mission1_step()
            elif self.mission_id == "mission2":
                self.execute_mission2_step()
            elif self.mission_id == "mission3":
                self.execute_mission3_step()

        #  (2) Check any pending service-call futures 
        for fut, success_msg, fail_msg in [
            (self.arm_future,      "[SERVICE] Arming succeeded",    "[SERVICE] Arming failed"),
            (self.disarm_future,   "[SERVICE] Disarming succeeded", "[SERVICE] Disarming failed"),
            (self.set_mode_future, "[SERVICE] SetMode succeeded",   "[SERVICE] SetMode failed"),
        ]:
            if fut is not None and fut.done():
                try:
                    res = fut.result()
                    if res.success:
                        rospy.loginfo(success_msg)
                    else:
                        rospy.logerr(fail_msg + " — returned success={}".format(res.success))
                except Exception as e:
                    rospy.logerr("[ERROR] Exception in future: {}".format(e))
                finally:
                    if fut is self.arm_future:
                        self.arm_future = None
                    if fut is self.disarm_future:
                        self.disarm_future = None
                    if fut is self.set_mode_future:
                        self.set_mode_future = None

    #
    #  MISSION1: ORIGINAL SEQUENCE 
    #
    def execute_mission1_step(self):
        """
        mission1:
          1) Set STABILIZE → wait
          2) Arm → wait
          3) Set FLOW_HOLD (mode 22) → wait
          4) Throttle override 10 s → LAND → 15 s delay → disarm
        """
        # Step 1: After requesting STABILIZE, wait until mode == "STABILIZE"
        if self.mission_step == 1 and self.set_mode_future is None and self.mode.upper() == "STABILIZE":
            self.m1_started = True
            rospy.loginfo("[MISSION1] STABILIZE confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2

        # Step 2: After arm_future completes and self.armed=True → switch to FLOW_HOLD
        elif self.mission_step == 2 and self.arm_future is None and self.armed:
            rospy.loginfo("[MISSION1] Armed → Switching to FLOW_HOLD...")
            self.set_mode("22")  # custom_mode 22 = FLOW_HOLD
            self.mission_step = 3

        # Step 3: Once FLOW_HOLD confirmed → start throttle override timer
        elif self.mission_step == 3 and self.mode.upper() == "CMODE(22)":
            rospy.loginfo("[MISSION1] Entered FLOW_HOLD → Throttle override for 10 s...")
            self.throttle_counter = 0
            self.mission_timer = self.create_timer(0.1, self.publish_throttle)
            self.mission_step = 4

        # Step 4: handled in publish_throttle()

    def publish_throttle(self):
        """
        Called at 10 Hz during mission1 Step 4:
          - Publish neutral throttle override (channel 3 = 1600)
          - After 100 iterations (10 s), cancel timer → LAND → start 15 s delay → disarm_after_landing()
        """
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        msg.channels[2] = 1650  # throttle neutral (adjust if needed)

        self.rc_override_pub.publish(msg)
        self.throttle_counter += 1

        if self.throttle_counter >= 1000:
            rospy.loginfo("[MISSION1] 10 s override complete → Switching to LAND")
            if self.mission_timer:
                self.mission_timer.cancel()
                self.mission_timer = None

            self._land_immediately()

            self.landing_delay_timer = self.create_timer(15.0, self.disarm_after_landing)
            self.mission_active = False
            self.mission_id = None
            self.mission_step = 0

    def disarm_after_landing(self):
        """
        Called 15 s after entering LAND in mission1 → send disarm.
        """
        rospy.loginfo("[MISSION1] 15 s LAND delay elapsed → Disarming now")
        self.disarm_drone()

        if self.landing_delay_timer:
            self.landing_delay_timer.cancel()
            self.landing_delay_timer = None
    #
    #  MISSION2: GUIDED TAKEOFF & BODY-FRAME MOVES 
    #
    def execute_mission2_step(self):

        #  Branch for “resume‑in‑air” behavior 
        if self.m2_resume:
            return self._execute_m2_resume()

        # mission2 original sequence
        if self.mission_step == 1 and self.set_mode_future is None and self.mode.upper() == "GUIDED":
            rospy.loginfo("[MISSION2] GUIDED confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2

        elif self.mission_step == 2 and self.arm_future is None and self.armed:
            # Send MAV_CMD_NAV_TAKEOFF via CommandLong
            try:
                req = CommandLong.Request()
                req.command = 22  # MAV_CMD_NAV_TAKEOFF
                req.param7 = float(self.M2_TAKEOFF_ALT)
                res = self.command_long_client(req)
                if res.success:
                    rospy.loginfo("[MISSION2] Takeoff to {} m requested.".format(self.M2_TAKEOFF_ALT))
                else:
                    rospy.logerr("[MISSION2] Takeoff command failed")
            except rospy.ServiceException as e:
                rospy.logerr("[MISSION2] Service call failed: {e}".format(e))
                return
            self.m2_takeoff_time = time.time()
            self.mission_step = 3

        elif self.mission_step == 3:
            if self.m2_takeoff_time and time.time() - self.m2_takeoff_time >= 10.0:
                if not self.m2_forward_sent:
                    rospy.loginfo("[MISSION2] Entering FORWARD phase → sending 3 m forward (body‑frame)…")
                    self.publish_body_offset(self.M2_FORWARD_DIST, 0.0)
                    self.m2_forward_sent = True
                if time.time() - self.m2_takeoff_time >= 20.0:
                    rospy.loginfo("[MISSION2] FORWARD phase COMPLETE → proceeding to SIDE phase.")
                    self.m2_side_start = time.time()
                    self.mission_step = 4

        elif self.mission_step == 4:
            elapsed = time.time() - self.m2_side_start
            if not self.m2_side_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering SIDE phase → sending 1 m right (body‑frame)…")
                self.publish_body_offset(0.0, self.M2_SIDE_DIST)
                self.m2_side_sent = True
            if elapsed >= 10.0:
                rospy.loginfo("[MISSION2] SIDE phase COMPLETE → proceeding to REVERSE phase.")
                self.m2_reverse_start = time.time()
                self.mission_step = 5

        elif self.mission_step == 5:
            elapsed = time.time() - self.m2_reverse_start
            if not self.m2_reverse_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering REVERSE phase → sending 3 m reverse (body‑frame)…")
                self.publish_body_offset(-self.M2_FORWARD_DIST, 0.0)
                self.m2_reverse_sent = True
            if elapsed >= 10.0:
                rospy.loginfo("[MISSION2] REVERSE phase COMPLETE → proceeding to LEFT phase.")
                self.m2_left_start = time.time()
                self.mission_step = 6

        elif self.mission_step == 6:
            elapsed = time.time() - self.m2_left_start
            if not self.m2_left_sent and elapsed >= 0.0:
                rospy.loginfo("[MISSION2] Entering LEFT phase → sending 1 m left (body‑frame)…")
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
        # Step 1: LOITRE mode has already been set
        if self.mission_step == 1:
            rospy.loginfo("[M2‑RESUME] LOITRE confirmed, starting throttle override…")
            self.mission_timer = rospy.Timer(rospy.Duration(0.1), lambda evt: self._publish_throttle_only())
            self.resume_start_time = time.time()
            self.mission_step = 2

        elif self.mission_step == 2 and time.time() - self.resume_start_time >= 5.0:
            rospy.loginfo("[M2‑RESUME] 5s throttle-only done → adding forward pitch")
            if self.mission_timer: self.mission_timer.shutdown()
            self.mission_timer = rospy.Timer(rospy.Duration(0.1), lambda evt: self._publish_throttle_and_pitch())
            self.move_start_time = time.time()
            self.mission_step = 3

        elif self.mission_step == 3 and time.time() - self.move_start_time >= 5.0:
            rospy.loginfo("[M2‑RESUME] forward move complete → FLOW_HOLD mode")
            if self.mission_timer: self.mission_timer.shutdown()
            self.set_mode("FLOW_HOLD")
            self.flowhold_start = time.time()
            self.mission_step = 4

        elif self.mission_step == 4 and time.time() - self.flowhold_start >= 5.0:
            rospy.loginfo("[M2‑RESUME] 5s in FLOW_HOLD done → LAND")
            self._land_immediately()
            rospy.Timer(rospy.Duration(15.0), lambda evt: self.disarm_after_landing_m2(), oneshot=True)
            self.mission_active = False
            self.mission_step = 0

    def _publish_throttle_only(self):
        msg = OverrideRCIn()
        msg.channels = [0]*18
        msg.channels[2] = 1650
        self.rc_override_pub.publish(msg)
        self._check_zero_pos_fallback()

    def _publish_throttle_and_pitch(self):
        msg = OverrideRCIn()
        msg.channels = [0]*18
        msg.channels[2] = 1650
        msg.channels[1] = 1600
        self.rc_override_pub.publish(msg)
        self._check_zero_pos_fallback()

    def _check_zero_pos_fallback(self):
        if not self.current_local_pose:
            return
        x = self.current_local_pose.pose.position.x
        y = self.current_local_pose.pose.position.y
        if x == 0.0 and y == 0.0 and self.mission_step in (2,3):
            rospy.logwarn("[M2‑RESUME] External nav lost → FLOW_HOLD fallback")
            if self.mission_timer: self.mission_timer.shutdown()
            now = time.time()
            if self.mission_step == 2:
                self.remaining_throttle_time = max(0.0, 5.0 - (now - self.resume_start_time))
            else:
                self.remaining_forward_time = max(0.0, 5.0 - (now - self.move_start_time))
            self.set_mode("FLOW_HOLD")
            self.flowhold_pause = True

    def disarm_after_landing_m2(self):
        rospy.loginfo("[MISSION2] Disarming after LAND.")
        self.disarm_drone()
        # no timer to cancel here

    #
    #  MISSION3: GUIDED TAKEOFF & BODY-FRAME MOVES WITH LAND/TAKEOFF BETWEEN EACH 
    #
    def execute_mission3_step(self):
        """
        mission3:
          1) Set GUIDED → wait until mode=="GUIDED" → arm
          2) After armed, send MAV_CMD_NAV_TAKEOFF to 2 m → record time → mission_step=3
          3) After ~10 s, publish body‐forward once → immediately LAND → wait 10 s → takeoff → mission_step=4
          4) After ~10 s (post-takeoff), publish body‐right once → immediately LAND → wait 10 s → takeoff → mission_step=5
          5) After ~10 s, publish body‐reverse once → immediately LAND → wait 10 s → takeoff → mission_step=6
          6) After ~10 s, publish body‐left once → immediately LAND → after 15 s, disarm → mission complete
        """

        # Step 1: After request GUIDED, wait until mode=="GUIDED", then arm
        if self.mission_step == 1 and self.set_mode_future is None and self.mode.upper() == "GUIDED":
            rospy.loginfo("[MISSION3] GUIDED confirmed → Arming now...")
            self.arm_drone()
            self.mission_step = 2

        # Step 2: After arm_future completes and self.armed=True → takeoff command
        elif self.mission_step == 2 and self.arm_future is None and self.armed:
            # Send MAV_CMD_NAV_TAKEOFF via CommandLong
            if not self.command_long_client.wait_for_service(1.0):
                rospy.logerr("CommandLong service not available for mission3.")
                return
            req = CommandLong.Request()
            req.command = 22                # MAV_CMD_NAV_TAKEOFF
            req.param7 = float(self.M3_TAKEOFF_ALT)
            self.command_long_client.call_async(req)

            rospy.loginfo("[MISSION3] Takeoff to {} m requested.".format(self.M3_TAKEOFF_ALT))
            self.m3_takeoff_time = time.time()
            self.mission_step = 3

        # Step 3: After ~10 s (allow to reach altitude), send forward, LAND, wait 10 s, then takeoff for next
        elif self.mission_step == 3:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_forward_sent:
                # Publish forward offset exactly once
                rospy.loginfo("[MISSION3] Entering FORWARD phase → sending 3 m forward (body‐frame)…")
                self.publish_body_offset(self.M3_FORWARD_DIST, 0.0)
                self.m3_forward_sent = True

                # Immediately land after sending forward
                rospy.loginfo("[MISSION3] FORWARD offset sent → Switching to LAND")
                self._land_immediately()

                # After 10 s on ground, call takeoff for “RIGHT” step
                self.landing_delay_timer = self.create_timer(self.M3_LAND_WAIT, self.m3_takeoff_for_side)
                # Don’t advance mission_step now; will be advanced in callback
       
        # Step 4 will be handled in m3_takeoff_for_side() after wait

        # Step 5: After takeoff for “RIGHT”, send right offset, LAND, wait, then takeoff for reverse
        elif self.mission_step == 5:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_side_sent:
                rospy.loginfo("[MISSION3] Entering SIDE phase → sending 1 m right (body‐frame)…")
                self.publish_body_offset(0.0, self.M3_SIDE_DIST)
                self.m3_side_sent = True

                rospy.loginfo("[MISSION3] SIDE offset sent → Switching to LAND")
                self._land_immediately()

                # After 10 s on ground, call takeoff for “REVERSE” step
                self.landing_delay_timer = self.create_timer(self.M3_LAND_WAIT, self.m3_takeoff_for_reverse)

        # Step 6 will be handled in m3_takeoff_for_reverse()

        # Step 7: After takeoff for “REVERSE”, send reverse offset, LAND, wait, then takeoff for left
        elif self.mission_step == 7:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_reverse_sent:
                rospy.loginfo("[MISSION3] Entering REVERSE phase → sending 3 m reverse (body‐frame)…")
                self.publish_body_offset(-self.M3_FORWARD_DIST, 0.0)
                self.m3_reverse_sent = True

                rospy.loginfo("[MISSION3] REVERSE offset sent → Switching to LAND")
                self._land_immediately()

                # After 10 s on ground, call takeoff for “LEFT” step
                self.landing_delay_timer = self.create_timer(self.M3_LAND_WAIT, self.m3_takeoff_for_left)

        # Step 8 will be handled in m3_takeoff_for_left()

        # Step 9: After takeoff for “LEFT”, send left offset, LAND, then disarm after 15 s
        elif self.mission_step == 9:
            elapsed = time.time() - self.m3_takeoff_time
            if elapsed >= 10.0 and not self.m3_left_sent:
                rospy.loginfo("[MISSION3] Entering LEFT phase → sending 1 m left (body‐frame)…")
                self.publish_body_offset(0.0, -self.M3_SIDE_DIST)
                self.m3_left_sent = True

                rospy.loginfo("[MISSION3] LEFT offset sent → Switching to LAND")
                self._land_immediately()

                # After 15 s on ground, disarm
                self.landing_delay_timer = self.create_timer(15.0, self.disarm_after_landing_m3)
                self.mission_active = False
                self.mission_id = None
                self.mission_step = 0

    #
    #  MISSION3 CALLBACKS: TAKEOFF FOR NEXT CORNER 
    #
    def m3_takeoff_for_side(self):
        """
        Called 10 s after landing from FORWARD move:
        → takeoff again and prepare for SIDE
        """
        # Cancel this timer
        if self.landing_delay_timer:
            self.landing_delay_timer.cancel()
            self.landing_delay_timer = None

        # Issue takeoff for SIDE
        if not self.command_long_client.wait_for_service(1.0):
            rospy.logerr("CommandLong service not available for mission3 (side).")
            return
        req = CommandLong.Request()
        req.command = 22                # MAV_CMD_NAV_TAKEOFF
        req.param7 = float(self.M3_TAKEOFF_ALT)
        self.command_long_client.call_async(req)

        rospy.loginfo("[MISSION3] Re‐Takeoff to {} m for SIDE phase requested.".format(self.M3_TAKEOFF_ALT))
        self.m3_takeoff_time = time.time()
        self.mission_step = 5  # move to SIDE step

    def m3_takeoff_for_reverse(self):
        """
        Called 10 s after landing from SIDE move:
        → takeoff again and prepare for REVERSE
        """
        # Cancel this timer
        if self.landing_delay_timer:
            self.landing_delay_timer.cancel()
            self.landing_delay_timer = None

        # Issue takeoff for REVERSE
        if not self.command_long_client.wait_for_service(1.0):
            rospy.logerr("CommandLong service not available for mission3 (reverse).")
            return
        req = CommandLong.Request()
        req.command = 22                # MAV_CMD_NAV_TAKEOFF
        req.param7 = float(self.M3_TAKEOFF_ALT)
        self.command_long_client.call_async(req)

        rospy.loginfo("[MISSION3] Re‐Takeoff to {} m for REVERSE phase requested.".format(self.M3_TAKEOFF_ALT))
        self.m3_takeoff_time = time.time()
        self.mission_step = 7  # move to REVERSE step

    def m3_takeoff_for_left(self):
        """
        Called 10 s after landing from REVERSE move:
        → takeoff again and prepare for LEFT
        """
        # Cancel this timer
        if self.landing_delay_timer:
            self.landing_delay_timer.cancel()
            self.landing_delay_timer = None

        # Issue takeoff for LEFT
        if not self.command_long_client.wait_for_service(1.0):
            rospy.logerr("CommandLong service not available for mission3 (left).")
            return
        req = CommandLong.Request()
        req.command = 22                # MAV_CMD_NAV_TAKEOFF
        req.param7 = float(self.M3_TAKEOFF_ALT)
        self.command_long_client.call_async(req)

        rospy.loginfo("[MISSION3] Re‐Takeoff to {} m for LEFT phase requested.".format(self.M3_TAKEOFF_ALT))
        self.m3_takeoff_time = time.time()
        self.mission_step = 9  # move to LEFT step

    def disarm_after_landing_m3(self):
        """
        Called ~15 s after final LEFT landing in mission3 → disarm.
        """
        rospy.loginfo("[MISSION3] Disarming after final LAND.")
        # Make sure your disarm_drone() calls the ServiceProxy synchronously
        self.disarm_drone()
        if hasattr(self, 'landing_delay_timer') and self.landing_delay_timer:
            self.landing_delay_timer.shutdown()
            self.landing_delay_timer = None


    #
    #  SHUTDOWN / SAFETY LAND HANDLER 
    #
    def safety_land_and_shutdown(self):
        """
        Attempt to switch to LAND before shutting down unexpectedly.
        """
        if self.mode.upper() != "LAND":
            rospy.logwarn("[SAFETY] Node shutting down unexpectedly → Forcing LAND")
            self._land_immediately()
            # Give the LAND command a moment to register
            rospy.sleep(0.1)


    #
    #  MAIN ENTRY POINT 
    #
if __name__ == "__main__":
    # 1) Initialize the ROS 1 node
    rospy.init_node('drone_controller')

    # 2) Instantiate your controller
    drone_controller = DroneController()

    try:
        # 3) Enter rospy’s event loop
        rospy.spin()

    except KeyboardInterrupt:
        # 4) Handle Ctrl‑C: log and safety land
        rospy.loginfo("[MAIN] KeyboardInterrupt detected → Safety landing")
        try:
            drone_controller.safety_land_and_shutdown()
        except Exception as e:
            rospy.logerr("[MAIN] Exception during safety landing: {}".format(e))

    except Exception as e:
        # 5) Catch-all for any other exception
        rospy.logerr("[MAIN] Unexpected exception: {} → Safety landing".format(e1))
        try:
            drone_controller.safety_land_and_shutdown()
        except Exception as e2:
            rospy.logerr("[MAIN] Exception during exception-handler landing: {}".format(e2))

    finally:
        # 6) Final shutdown message
        rospy.loginfo("[MAIN] Shutting down node")
        # (No explicit destroy_node or shutdown needed; rospy.spin() will exit cleanly)

