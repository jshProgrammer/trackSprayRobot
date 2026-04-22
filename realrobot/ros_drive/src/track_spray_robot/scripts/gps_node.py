#!/usr/bin/env python3
import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus

def gps_publisher():
    # Initialisiere den ROS Node
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    rospy.init_node('lc29h_driver', anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)

    frame_id = rospy.get_param("~frame_id", "gps_link")

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.5)
        rospy.loginfo(f"LC29H verbunden auf {port}")

    except Exception as e:
        rospy.logerr(f"Serieller Fehler: {e}")
        return

    while not rospy.is_shutdown():
        line = ser.readline().decode('ascii', errors='replace')
        if line.startswith('$GNGGA'):
            
            msg = pynmea2.parse(line)

            fix = NavSatFix()
            fix.header.stamp = rospy.Time.now()
            fix.header.frame_id = frame_id
                
            # =========================
            # GPS QUALITY (RTK STATUS)
            # =========================
            gps_qual = int(msg.gps_qual)

            # Mapping RTK Status
            if gps_qual == 4:
                fix.status.status = NavSatStatus.STATUS_FIX  # RTK FIX
                rospy.loginfo_throttle(1, "RTK FIX (cm-level)")

            elif gps_qual == 5:
                fix.status.status = NavSatStatus.STATUS_SBAS_FIX  # RTK FLOAT
                rospy.logwarn_throttle(1, "RTK FLOAT")

            elif gps_qual > 0:
                fix.status.status = NavSatStatus.STATUS_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
                rospy.logwarn_throttle(5, "No GPS Fix")
                continue

            # =========================
            # POSITION
            # =========================
            fix.latitude = float(msg.latitude)
            fix.longitude = float(msg.longitude)
            fix.altitude = float(msg.altitude) if msg.altitude else 0.0

            # =========================
            # COVARIANCE (RTK important!)
            # =========================
            fix.position_covariance = [0.0] * 9

            if gps_qual == 4:
                fix.position_covariance[0] = 0.01  # ~cm precision
            elif gps_qual == 5:
                fix.position_covariance[0] = 0.1   # float
            else:
                fix.position_covariance[0] = 5.0   # normal GPS

            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # =========================
            # PUBLISH
            # =========================
            pub.publish(fix)

            #rospy.logwarn_throttle(10, "Warte auf GPS-Fix...")
            #except pynmea2.ParseError:

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
