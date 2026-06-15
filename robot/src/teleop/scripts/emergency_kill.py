#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
import os
from teleop.msg import EmergencyReset

class EmergencyHandler:
    def __init__(self):
        rospy.init_node("emergency_handler", disable_signals=True)
        
        # Hard Stop - kill all
        rospy.Subscriber("/emergency_reset", EmergencyReset, self.stop)
        
        rospy.loginfo("Emergency Handler started")

    def stop(self, msg):
        if msg.is_soft:
            self.soft_reset()
        else:
            self.hard_stop()

    def hard_stop(self):
        rospy.logfatal("❌ HARD EMERGENCY STOP - KILLING EVERYTHING!")

        # Killt alle Fahr-/Navigationsprozesse. localization (gps_node/ntrip_client),
        # roscore und rosbridge bleiben bewusst am Leben, damit das Frontend verbunden
        # bleibt und nach einem Not-Aus wieder Phase 2 starten kann.
        os.system(
            "pkill -f 'pwm_drive.py|kalman_filter_2.5D_optimized.py|navigation.py|"
            "obstacle_avoidance_node.py|joy_node|teleop_node|joystick.py' &"
        )

        rospy.signal_shutdown("Hard emergency stop")

    def soft_reset(self):
        rospy.logwarn("🔄 SOFT RESET - Restarting motor driver only")
        
        # Only kill pwm_drive.py (will be respawned automatically due to launch file)
        os.system("pkill -f 'pwm_drive.py' &")

if __name__ == "__main__":
    EmergencyHandler()
    rospy.spin()