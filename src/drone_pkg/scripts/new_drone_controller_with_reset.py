#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from my_msgs.msg import GridDetection
from mavros_msgs.msg import State, PositionTarget, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest, CommandLongRequest
from rtabmap_ros.srv import ResetPose
from tf.transformations import euler_from_quaternion

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')

        # State Variables
        self.connected = False
        self.armed = False
        self.mode = ""

        # Border detection
        self.top_grid_detected = False
        self.bottom_grid_detected = False
        self.left_grid_detected = False 
        self.right_grid_detected = False
        self.center_grid_detected = False

        # Position
        self.current_pose = None
        self.last_pose = None # To reset odom with last pose if tracking stops

        # Subscriptions
        rospy.Subscriber("/manual_drone_cmds", String, self.manual_drone_cmd_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("yellow_grid", GridDetection, self.grid_cb)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_cb)

        # Publishers
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.setpnt_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.command_long_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        rospy.loginfo("DroneController initialized")

    def manual_drone_cmd_cb(self, msg):
        """
        Accepts:
          - "arm"
          - "disarm"
          - "mode <MODE_NAME>"
          - "land" or "l"
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
        elif cmd in ("land", "l"):
            self._land_immediately()
        else:
            rospy.logwarn("Unknown manual command: '{}'".format(cmd))

    def state_cb(self, msg):
        """
        Update connected, armed status, and current mode.
        """
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        rospy.loginfo("STATE: [ {} / {} / {} ]".format(
            'Connected' if self.connected else 'Not connected',
            'Armed' if self.armed else 'Disarmed',
            self.mode))

    def grid_cb(self, msg):
        self.top_grid_detected = all(msg.row1)
        self.bottom_grid_detected = all(msg.row3)
        self.left_grid_detected = all([msg.row1[0], msg.row2[0], msg.row3[0]])
        self.right_grid_detected = all([msg.row1[-1], msg.row2[-1], msg.row3[-1]])
        self.center_grid_detected = msg.row2[1] == 1

        if any([self.top_grid_detected, self.bottom_grid_detected, 
               self.left_grid_detected, self.right_grid_detected, 
               self.center_grid_detected]):
            rospy.loginfo("YELLOW BORDER DETECTED. Mode set to FLOW_HOLD")
            self.set_mode("22") # flowhold

    def pose_cb(self, msg):
        self.current_pose = msg.pose 
        if not all([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]):
          rospy.logwarn("[ERROR] BAD VISUAL ODOM")
          self.set_mode("22")
          if self.last_pose:
            self.reset_odom_by_pose(self.last_pose)
        else:
          self.last_pose = self.current_pose

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

          

    def arm_drone(self):
        try:
            req = CommandBoolRequest()
            req.value = True
            resp = self.arming_client(req)
            if resp.success:
                rospy.loginfo("[SERVICE] Arming succeeded")
            else:
                rospy.logerr("[SERVICE] Arming failed")
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] Arming service call failed: {}".format(e))

    def disarm_drone(self):
        try:
            req = CommandBoolRequest()
            req.value = False
            resp = self.arming_client(req)
            if resp.success:
                rospy.loginfo("[SERVICE] Disarming succeeded")
            else:
                rospy.logerr("[SERVICE] Disarming failed")
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] Disarming service call failed: {}".format(e))

    def set_mode(self, mode):
        try:
            req = SetModeRequest()
            req.custom_mode = mode.upper()
            resp = self.mode_client(req)
            if resp.mode_sent:
                rospy.loginfo("[SERVICE] SetMode({}) succeeded".format(mode.upper()))
            else:
                rospy.logerr("[SERVICE] SetMode({}) failed".format(mode.upper()))
        except rospy.ServiceException as e:
            rospy.logerr("[SERVICE] SetMode service call failed: {}".format(e))

    def _land_immediately(self):
        rospy.logwarn("[SAFETY] Forcing LAND mode now!")
        self.set_mode("LAND")

    def safety_land_and_shutdown(self):
        if self.mode.upper() != "LAND":
            rospy.logwarn("[SAFETY] Emergency landing.")
            self.set_mode("LAND")
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        controller = DroneController()
        rospy.spin()
    except rospy.ROSInterruptException:
        try:
            controller.safety_land_and_shutdown()
        except:
            pass
    except Exception as e:
        rospy.logerr("Unexpected exception: {}".format(e))
        try:
            controller.safety_land_and_shutdown()
        except:
            pass
