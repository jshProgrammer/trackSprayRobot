#!/usr/bin/env python3

import rospy
from threading import Thread
from ntrip_client_lib import NtripClient  

class NtripRosNode:

    def __init__(self):
        rospy.init_node('ntrip_client_node')

        # ROS Parameter
        caster = rospy.get_param('~caster')
        port = rospy.get_param('~port', 2101)
        mountpoint = rospy.get_param('~mountpoint')
        user = rospy.get_param('~user')
        password = rospy.get_param('~password')

        serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        serial_baud = rospy.get_param("~serial_baud", 115200)

        latitude = rospy.get_param('~lat', 50.0)
        longitude = rospy.get_param('~lon', 10.0)

        # NTRIP Client
        self.client = NtripClient(
            caster=caster,
            port=port,
            mountpoint=mountpoint,
            user=f"{user}:{password}",
            lat=latitude,
            lon=longitude,
            verbose=True,
            V2=True,
            serial_port = serial_port,
            serial_baud = serial_baud
        )

        # Thread starten (wichtig, sonst blockiert ROS)
        self.thread = Thread(target=self.client.readData)
        self.thread.daemon = True
        self.thread.start()

        rospy.loginfo("NTRIP Client gestartet")

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = NtripRosNode()
    node.spin()
