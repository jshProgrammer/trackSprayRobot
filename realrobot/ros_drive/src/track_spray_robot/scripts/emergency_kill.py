#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
import os

class EmergencyHandler:
    def __init__(self):
        rospy.init_node("emergency_handler", disable_signals=True)
        
        # Hard Stop - kill all
        rospy.Subscriber("/emergency_stop", Empty, self.hard_stop)
        
        # Soft Reset - only restart motor pwm_drive node
        rospy.Subscriber("/soft_reset", Empty, self.soft_reset)
        
        rospy.loginfo("Emergency Handler started")

    def hard_stop(self, msg):
        rospy.logfatal("❌ HARD EMERGENCY STOP - KILLING EVERYTHING!")
        
        os.system("pkill -f 'pwm_drive.py|joy_node|teleop_node|joystick.py|ntrip_client' &")
        
        rospy.signal_shutdown("Hard emergency stop")

    def soft_reset(self, msg):
        rospy.logwarn("🔄 SOFT RESET - Restarting motor driver only")
        
        # Only kill pwm_drive.py (will be respawned automatically due to launch file)
        os.system("pkill -f 'pwm_drive.py' &")

if __name__ == "__main__":
    EmergencyHandler()
    rospy.spin()