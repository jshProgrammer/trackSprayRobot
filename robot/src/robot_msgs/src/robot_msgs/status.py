"""StatusReporter – zentraler Helfer, um RobotStatus-Events ans Frontend zu publishen.

Alle Nodes publishen auf dasselbe Topic ``/robot_status`` (latched). Das Frontend
abonniert nur dieses eine Topic und filtert über ``source`` / ``type`` / ``code``.

- ``latch=True``: ein spät verbundenes Frontend (rosbridge-Reconnect) bekommt den
  letzten Status sofort.
- ``report(..., dedup=True)``: edge-triggered – nur publishen, wenn sich der ``code``
  gegenüber dem letzten Event geändert hat (kein Spam in High-Rate-Loops).
- ``report_fatal``: latched publish + kurzer Flush-Sleep, BEVOR die Node stirbt, damit
  die Nachricht die Subscriber/rosbridge garantiert erreicht.
"""

import rospy
from robot_msgs.msg import RobotStatus

# Flush-Zeit, damit ein latched publish einer gleich sterbenden Node noch rausgeht.
_FATAL_FLUSH_SEC = 0.3


class StatusReporter:
    def __init__(self, source, topic="/robot_status"):
        self.source = source
        self.pub = rospy.Publisher(topic, RobotStatus, queue_size=10, latch=True)
        self._last_code = None

    def report(self, type, code, message, dedup=True):
        """Published ein Status-Event. dedup=True unterdrückt Wiederholungen desselben code."""
        if dedup and code == self._last_code:
            return
        self._last_code = code
        self.pub.publish(RobotStatus(
            stamp=rospy.Time.now(),
            type=type,
            source=self.source,
            code=code,
            message=message,
        ))

    def info(self, code, message, dedup=True):
        self.report("info", code, message, dedup=dedup)

    def warn(self, code, message, dedup=True):
        self.report("warn", code, message, dedup=dedup)

    def error(self, code, message, dedup=True):
        self.report("error", code, message, dedup=dedup)

    def report_fatal(self, code, message):
        """Für Fehler, nach denen die Node stirbt: publish (latched) + kurzer Flush-Sleep."""
        self.report("error", code, message, dedup=False)
        try:
            rospy.sleep(_FATAL_FLUSH_SEC)
        except rospy.ROSInterruptException:
            pass
