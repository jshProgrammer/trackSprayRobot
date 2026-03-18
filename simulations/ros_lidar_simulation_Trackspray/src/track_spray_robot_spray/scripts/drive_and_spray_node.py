#!/usr/bin/env python3
"""
drive_and_spray_node.py
=======================
Faehrt geradeaus, stoppt alle 2 m (Odometrie), betaetigt Spray-Aktuator,
spawnt orangen Farbfleck in Gazebo.
"""

import math
import rospy
from nav_msgs.msg            import Odometry
from geometry_msgs.msg       import Twist, Point, Pose
from sensor_msgs.msg         import JointState
from visualization_msgs.msg  import Marker, MarkerArray
from trajectory_msgs.msg     import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv         import SpawnModel
from gazebo_msgs.msg         import ModelState

# ------------------------------------------------------------------ #
#  Konfiguration                                                       #
# ------------------------------------------------------------------ #
DRIVE_DISTANCE    = 2.0    # m   – Abstand zwischen Spray-Stopps
FORWARD_VELOCITY  = 0.5    # m/s

SPRAY_JOINT_NAME  = "spray_joint"
STROKE_DOWN_M     = 0.10   # m   – Stab ausgefahren
STROKE_UP_M       = 0.0    # m   – Stab eingezogen
STROKE_DURATION_S = 0.6    # s   – Zeit pro Bewegungsrichtung
SPRAY_DWELL_TIME  = 1.0    # s   – Wie lange Stab unten bleibt

MARKER_SCALE      = 0.12   # m   – Durchmesser Bodenmarkierung
MARKER_HEIGHT     = 0.008  # m   – Dicke Bodenmarkierung
# ------------------------------------------------------------------ #

# Zustandsmaschine
STATE_DRIVING      = 0
STATE_STROKE_DOWN  = 1
STATE_DWELL        = 2
STATE_STROKE_UP    = 3

# SDF-Template fuer orangen Farbfleck in Gazebo
MARKER_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.4 0.0 1.0</ambient>
          <diffuse>1.0 0.4 0.0 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class DriveAndSprayNode:

    def __init__(self):
        rospy.init_node('drive_and_spray', anonymous=False)

        # --- Publisher ---
        self.cmd_pub    = rospy.Publisher('/cmd_vel',        Twist,          queue_size=1)
        self.js_pub     = rospy.Publisher('/joint_states',   JointState,     queue_size=5)
        self.traj_pub   = rospy.Publisher('/spray_controller/command',
                                          JointTrajectory,  queue_size=3)
        # RViz Marker (optional, zusaetzlich)
        self.marker_pub = rospy.Publisher('/ground_markers', MarkerArray,    queue_size=5)

        # --- Subscriber ---
        rospy.Subscriber('/odom', Odometry, self._odom_callback, queue_size=10)

        # --- Gazebo SpawnModel Service ---
        rospy.loginfo("drive_and_spray: warte auf /gazebo/spawn_sdf_model ...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.loginfo("drive_and_spray: Gazebo spawn service bereit")

        # --- Odometrie ---
        self.last_x          = None
        self.last_y          = None
        self.dist_since_stop = 0.0

        # --- Zustandsmaschine ---
        self.state           = STATE_DRIVING
        self.state_start     = None
        self.spray_position  = Point()

        # --- Aktuator ---
        self.joint_position  = STROKE_UP_M

        # --- Marker ---
        self.marker_id       = 0
        self.marker_array    = MarkerArray()

        rospy.loginfo("drive_and_spray: gestartet – stoppe alle %.1f m", DRIVE_DISTANCE)

    # ---------------------------------------------------------------- #
    #  Odometrie-Callback                                               #
    # ---------------------------------------------------------------- #

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:
            self.last_x = x
            self.last_y = y
            return

        if self.state == STATE_DRIVING:
            dx   = x - self.last_x
            dy   = y - self.last_y
            step = math.sqrt(dx * dx + dy * dy)
            self.dist_since_stop += step

            if self.dist_since_stop >= DRIVE_DISTANCE:
                self.dist_since_stop = 0.0
                self.spray_position  = Point(x=x, y=y, z=0.0)
                self._enter_state(STATE_STROKE_DOWN)
                rospy.loginfo("drive_and_spray: %.1f m erreicht – Spray-Zyklus startet", DRIVE_DISTANCE)

        self.last_x = x
        self.last_y = y

    # ---------------------------------------------------------------- #
    #  Zustandsmaschine                                                 #
    # ---------------------------------------------------------------- #

    def _enter_state(self, new_state):
        self.state       = new_state
        self.state_start = rospy.Time.now()

        if new_state == STATE_STROKE_DOWN:
            self._move_actuator(STROKE_DOWN_M)
            rospy.loginfo("drive_and_spray: Stab faehrt AUS")

        elif new_state == STATE_DWELL:
            rospy.loginfo("drive_and_spray: Stab unten – Marker setzen")
            self._spawn_gazebo_marker(self.spray_position)
            self._add_rviz_marker(self.spray_position)

        elif new_state == STATE_STROKE_UP:
            self._move_actuator(STROKE_UP_M)
            rospy.loginfo("drive_and_spray: Stab faehrt EIN")

        elif new_state == STATE_DRIVING:
            rospy.loginfo("drive_and_spray: Weiterfahren")

    def _update_state(self):
        if self.state == STATE_DRIVING or self.state_start is None:
            return

        elapsed = (rospy.Time.now() - self.state_start).to_sec()

        if self.state == STATE_STROKE_DOWN and elapsed >= STROKE_DURATION_S:
            self._enter_state(STATE_DWELL)

        elif self.state == STATE_DWELL and elapsed >= SPRAY_DWELL_TIME:
            self._enter_state(STATE_STROKE_UP)

        elif self.state == STATE_STROKE_UP and elapsed >= STROKE_DURATION_S:
            self._enter_state(STATE_DRIVING)

    # ---------------------------------------------------------------- #
    #  Gazebo Marker spawnen                                            #
    # ---------------------------------------------------------------- #

    def _spawn_gazebo_marker(self, position: Point):
        name = "spray_marker_{}".format(self.marker_id)
        sdf  = MARKER_SDF.format(
            name=name,
            radius=MARKER_SCALE / 2.0,
            height=MARKER_HEIGHT
        )

        pose = Pose()
        pose.position.x    = position.x
        pose.position.y    = position.y
        pose.position.z    = MARKER_HEIGHT / 2.0
        pose.orientation.w = 1.0

        try:
            self.spawn_model(
                model_name=name,
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world"
            )
            rospy.loginfo("drive_and_spray: Gazebo-Marker '%s' bei (%.2f, %.2f) gespawnt",
                          name, position.x, position.y)
        except Exception as e:
            rospy.logwarn("drive_and_spray: Spawn fehlgeschlagen: %s", str(e))

    # ---------------------------------------------------------------- #
    #  RViz Marker (zusaetzlich)                                        #
    # ---------------------------------------------------------------- #

    def _add_rviz_marker(self, position: Point):
        m                    = Marker()
        m.header.frame_id    = "odom"
        m.header.stamp       = rospy.Time.now()
        m.ns                 = "ground_markers"
        m.id                 = self.marker_id
        m.type               = Marker.CYLINDER
        m.action             = Marker.ADD
        m.pose.position.x    = position.x
        m.pose.position.y    = position.y
        m.pose.position.z    = MARKER_HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x            = MARKER_SCALE
        m.scale.y            = MARKER_SCALE
        m.scale.z            = MARKER_HEIGHT
        m.color.r            = 1.0
        m.color.g            = 0.4
        m.color.b            = 0.0
        m.color.a            = 0.95
        m.lifetime           = rospy.Duration(0)

        self.marker_array.markers.append(m)
        self.marker_pub.publish(self.marker_array)
        rospy.loginfo("drive_and_spray: Marker #%d bei (%.2f, %.2f)",
                      self.marker_id, position.x, position.y)
        self.marker_id += 1

    # ---------------------------------------------------------------- #
    #  Aktuator                                                         #
    # ---------------------------------------------------------------- #

    def _move_actuator(self, target: float):
        self.joint_position = target
        self._publish_joint_state()
        self._send_trajectory_command(target)

    def _publish_joint_state(self):
        js              = JointState()
        js.header.stamp = rospy.Time.now()
        js.name         = [SPRAY_JOINT_NAME]
        js.position     = [self.joint_position]
        js.velocity     = [0.0]
        js.effort       = [0.0]
        self.js_pub.publish(js)

    def _send_trajectory_command(self, position: float):
        traj               = JointTrajectory()
        traj.header.stamp  = rospy.Time.now()
        traj.joint_names   = [SPRAY_JOINT_NAME]
        pt                 = JointTrajectoryPoint()
        pt.positions       = [position]
        pt.velocities      = [0.0]
        pt.time_from_start = rospy.Duration(STROKE_DURATION_S)
        traj.points        = [pt]
        self.traj_pub.publish(traj)

    # ---------------------------------------------------------------- #
    #  Haupt-Loop                                                       #
    # ---------------------------------------------------------------- #

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._update_state()

            cmd = Twist()
            if self.state == STATE_DRIVING:
                cmd.linear.x = FORWARD_VELOCITY
            else:
                cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = DriveAndSprayNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("drive_and_spray: beendet")