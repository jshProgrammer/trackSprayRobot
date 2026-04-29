#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class JoyToCmdVel:
    def __init__(self):
        rospy.init_node("joy_to_cmdvel")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.spray_pub = rospy.Publisher("/cmd_spray", Empty, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.callback)

        self.max_linear = rospy.get_param("~max_linear", 0.5)
        self.max_angular = rospy.get_param("~max_angular", 1.0)

        rospy.loginfo("Joy to cmd_vel node started")

    def callback(self, joy):
        twist = Twist()

        axes = joy.axes
        buttons = joy.buttons

        # -------------------
        # STEERING (left stick X)
        # axes[0] laut deiner Beschreibung
        # -------------------
        steer = axes[0]  # +1 links, -1 rechts
        twist.angular.z = steer * self.max_angular

        # -------------------
        # GAS (R2 = axis[4])
        # 1.0 -> 0 (nicht gedrückt)
        # -1.0 -> 1 (voll gedrückt)
        # -------------------
        r2 = axes[4]
        gas = (1.0 - r2) / 2.0
        twist.linear.x = gas * self.max_linear

        self.pub.publish(twist)

        spray = buttons[1]
        if spray == 1:
            self.spray_pub.publish()

if __name__ == "__main__":
    JoyToCmdVel()
    rospy.spin()