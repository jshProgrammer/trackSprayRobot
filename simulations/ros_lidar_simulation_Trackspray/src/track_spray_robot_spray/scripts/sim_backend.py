#!/usr/bin/env python3
"""
sim_backend.py
==============

Sim-Implementierungen von den Interfaces
"""

import rospy
from geometry_msgs.msg       import Twist, Point, Pose
from sensor_msgs.msg         import JointState
from visualization_msgs.msg  import Marker, MarkerArray
from trajectory_msgs.msg     import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv         import SpawnModel

from interfaces import DriveInterface, ActuatorInterface, MarkerInterface

# SDF-Template – unveraendert aus der Originalversion
_MARKER_SDF = """
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

_MARKER_SCALE  = 0.12   # m – Durchmesser
_MARKER_HEIGHT = 0.008  # m – Dicke


class SimDrive(DriveInterface):
    """Sendet Twist-Nachrichten auf /cmd_vel."""

    def __init__(self):
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def set_velocity(self, linear_x: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear_x
        self._pub.publish(cmd)


class SimActuator(ActuatorInterface):
    """
    Steuert den Spray-Joint ueber /joint_states und
    /spray_controller/command (JointTrajectory).
    """

    JOINT_NAME = rospy.get_param('/spray_joint_name', 'spray_joint') \
        if rospy.has_param('/spray_joint_name') else 'spray_joint'

    def __init__(self):
        self._js_pub   = rospy.Publisher('/joint_states',
                                         JointState,      queue_size=5)
        self._traj_pub = rospy.Publisher('/spray_controller/command',
                                         JointTrajectory, queue_size=3)
        self._position = 0.0

    def move_to(self, position: float, duration: float) -> None:
        self._position = position
        self._publish_joint_state()
        self._send_trajectory(position, duration)

    # ---------------------------------------------------------------- #

    def _publish_joint_state(self):
        js              = JointState()
        js.header.stamp = rospy.Time.now()
        js.name         = [self.JOINT_NAME]
        js.position     = [self._position]
        js.velocity     = [0.0]
        js.effort       = [0.0]
        self._js_pub.publish(js)

    def _send_trajectory(self, position: float, duration: float):
        traj               = JointTrajectory()
        traj.header.stamp  = rospy.Time.now()
        traj.joint_names   = [self.JOINT_NAME]
        pt                 = JointTrajectoryPoint()
        pt.positions       = [position]
        pt.velocities      = [0.0]
        pt.time_from_start = rospy.Duration(duration)
        traj.points        = [pt]
        self._traj_pub.publish(traj)


class SimMarker(MarkerInterface):
    """
    Platziert Gazebo-Modelle (oranger Zylinder) und
    veroeffentlicht zusaetzlich RViz-Marker auf /ground_markers.
    """

    def __init__(self):
        rospy.loginfo("sim_backend: warte auf /gazebo/spawn_sdf_model ...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self._spawn      = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                              SpawnModel)
        self._marker_pub = rospy.Publisher('/ground_markers',
                                           MarkerArray, queue_size=5)
        self._marker_array = MarkerArray()
        rospy.loginfo("sim_backend: Gazebo spawn service bereit")

    def place_marker(self, position: Point, marker_id: int) -> None:
        self._spawn_gazebo(position, marker_id)
        self._add_rviz_marker(position, marker_id)

    # ---------------------------------------------------------------- #

    def _spawn_gazebo(self, position: Point, marker_id: int):
        name = "spray_marker_{}".format(marker_id)
        sdf  = _MARKER_SDF.format(
            name=name,
            radius=_MARKER_SCALE / 2.0,
            height=_MARKER_HEIGHT,
        )
        pose = Pose()
        pose.position.x    = position.x
        pose.position.y    = position.y
        pose.position.z    = _MARKER_HEIGHT / 2.0
        pose.orientation.w = 1.0
        try:
            self._spawn(
                model_name=name,
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world",
            )
            rospy.loginfo("sim_backend: Gazebo-Marker '%s' bei (%.2f, %.2f)",
                          name, position.x, position.y)
        except Exception as exc:
            rospy.logwarn("sim_backend: Spawn fehlgeschlagen: %s", exc)

    def _add_rviz_marker(self, position: Point, marker_id: int):
        m                    = Marker()
        m.header.frame_id    = "odom"
        m.header.stamp       = rospy.Time.now()
        m.ns                 = "ground_markers"
        m.id                 = marker_id
        m.type               = Marker.CYLINDER
        m.action             = Marker.ADD
        m.pose.position.x    = position.x
        m.pose.position.y    = position.y
        m.pose.position.z    = _MARKER_HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x            = _MARKER_SCALE
        m.scale.y            = _MARKER_SCALE
        m.scale.z            = _MARKER_HEIGHT
        m.color.r            = 1.0
        m.color.g            = 0.4
        m.color.b            = 0.0
        m.color.a            = 0.95
        m.lifetime           = rospy.Duration(0)
        self._marker_array.markers.append(m)
        self._marker_pub.publish(self._marker_array)