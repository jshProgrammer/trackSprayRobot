#!/usr/bin/env python3
import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus

def gps_publisher():
    # Initialisiere den ROS Node
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    rospy.init_node('lc29h_driver', anonymous=True)

    port = "/dev/ttyUSB0"
    baud = 115200

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.5)
        rospy.loginfo(f"LC29H verbunden auf {port}")

        while not rospy.is_shutdown():
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GNRMC'):
                try:
                    msg = pynmea2.parse(line)
                    if msg.status == 'A':
                        # ROS Nachricht befüllen
                        fix = NavSatFix()
                        fix.header.stamp = rospy.Time.now()
                        fix.header.frame_id = "gps_link"

                        fix.status.status = NavSatStatus.STATUS_FIX
                        fix.latitude = msg.latitude
                        fix.longitude = msg.longitude

                        # Position veröffentlichen
                        pub.publish(fix)
                    else:
                        rospy.logwarn_throttle(10, "Warte auf GPS-Fix...")
                except pynmea2.ParseError:
                    continue
    except Exception as e:
        rospy.logerr(f"Serieller Fehler: {e}")

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
