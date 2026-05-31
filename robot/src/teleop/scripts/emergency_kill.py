#!/usr/bin/env python3

import rospy
import subprocess
import sys
from teleop.msg import EmergencyReset

class EmergencyHandler:
    def __init__(self):
        rospy.init_node("emergency_handler", disable_signals=True)
        rospy.Subscriber("/emergency_reset", EmergencyReset, self.stop)
        rospy.loginfo("Emergency Handler started")

    def stop(self, msg):
        if msg.is_soft:
            self.soft_reset()
        else:
            self.hard_stop()

    def _kill_process(self, pattern):
        """Kill processes matching pattern using subprocess (safer than os.system)"""
        try:
            # Use pgrep to find PIDs, then kill specifically by PID to avoid matching unrelated processes
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0 and result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    try:
                        subprocess.run(["kill", "-9", pid], timeout=1)
                        rospy.loginfo(f"Killed process {pid}")
                    except Exception as e:
                        rospy.logerr(f"Failed to kill PID {pid}: {e}")
            else:
                rospy.logwarn(f"No processes matching pattern '{pattern}' found")
        except subprocess.TimeoutExpired:
            rospy.logerr("Process lookup timed out")
        except Exception as e:
            rospy.logerr(f"Error killing processes: {e}")

    def hard_stop(self):
        rospy.logfatal("❌ HARD EMERGENCY STOP - KILLING EVERYTHING!")

        # Kill critical motor control nodes
        self._kill_process("pwm_drive.py")
        self._kill_process("joystick.py")
        self._kill_process("ntrip_client.py")
        self._kill_process("gps_node.py")

        rospy.signal_shutdown("Hard emergency stop")

    def soft_reset(self):
        rospy.logwarn("🔄 SOFT RESET - Restarting motor driver only")
        self._kill_process("pwm_drive.py")

if __name__ == "__main__":
    EmergencyHandler()
    rospy.spin()