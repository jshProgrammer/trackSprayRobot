#!/usr/bin/env python3
import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import ByteMultiArray, String

class LC29HDriver:
    def __init__(self):
        rospy.init_node('lc29h_driver', anonymous=True)
 
        self._init_params()
        self._init_serial()
        self._init_publishers()
        self._init_subscribers()
 
    # =========================
    # Receiving parameters from config
    # =========================
    def _init_params(self):
        self.port     = rospy.get_param("~port",     "/dev/ttyUSB0")
        self.baud     = rospy.get_param("~baud",     115200)
        self.frame_id = rospy.get_param("~frame_id", "gps_link")
 
    def _init_serial(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.5)
            rospy.loginfo(f"LC29H connected on {self.port}")
        except Exception as e:
            rospy.logerr(f"Serial error: {e}")
            raise

    # =========================
    # Publisher initialization
    # =========================
    def _init_publishers(self):
        #TODO: probably only use gps in the futures (with fixed and unfixed together), and then just filter
        self.pub         = rospy.Publisher('gps/fix',           NavSatFix, queue_size=10)
        # unfixed publisher will probably be removed in the future!
        self.unfixed_pub = rospy.Publisher('gps/unfixed',       NavSatFix, queue_size=10)
        self.nmea_pub    = rospy.Publisher('gps/nmea_sentence', String,    queue_size=10)
 
    def _init_subscribers(self):
        rospy.Subscriber('gps/rtcm', ByteMultiArray, self._rtcm_callback)
 
    # =========================
    # RTCM Callback (for NTRIP Client / RTK)
    # =========================
    def _rtcm_callback(self, msg):
        try:
            self.ser.write(bytes(msg.data))
        except Exception as e:
            rospy.logerr_throttle(5, f"RTCM write error: {e}")

    # =========================
    # Unfixed GPS (for debugging)
    # =========================
    def _build_unfixed(self, msg) -> NavSatFix:
        unfixed = NavSatFix()
        unfixed.header.stamp    = rospy.Time.now()
        unfixed.header.frame_id = self.frame_id
 
        unfixed.latitude  = float(msg.latitude)
        unfixed.longitude = float(msg.longitude)
        unfixed.altitude  = float(msg.altitude) if msg.altitude else 0.0
 
        unfixed.status.status  = NavSatStatus.STATUS_NO_FIX
        unfixed.status.service = NavSatStatus.SERVICE_GPS
 
        return unfixed
 
    def _build_fix(self, msg, gps_qual: int) -> NavSatFix:
        fix = NavSatFix()
        fix.header.stamp    = rospy.Time.now()
        fix.header.frame_id = self.frame_id
 
        # =========================
        # GPS QUALITY (RTK STATUS)
        # =========================
        if gps_qual == 4:
            fix.status.status = NavSatStatus.STATUS_FIX       # RTK FIX
            rospy.loginfo_throttle(1, "RTK FIX (cm-level)")
        elif gps_qual == 5:
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX  # RTK FLOAT
            rospy.logwarn_throttle(1, "RTK FLOAT")
        elif gps_qual > 0:
            fix.status.status = NavSatStatus.STATUS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            rospy.logwarn_throttle(5, "No GPS Fix")
            return None
 
        # =========================
        # POSITION
        # =========================
        fix.latitude  = float(msg.latitude)
        fix.longitude = float(msg.longitude)
        fix.altitude  = float(msg.altitude) if msg.altitude else 0.0
 
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
 
        return fix
 
    def _process_line(self, line: str):
        # pass all NMEA sentences for NTRIP_Client
        if line.startswith('$'):
            self.nmea_pub.publish(String(data=line))
 
        if not line.startswith('$GNGGA'):
            return
 
        try:
            msg      = pynmea2.parse(line)
            gps_qual = int(msg.gps_qual)
 
            self.unfixed_pub.publish(self._build_unfixed(msg))
 
            # =========================
            # PUBLISH
            # =========================
            fix = self._build_fix(msg, gps_qual)
            if fix is not None:
                self.pub.publish(fix)
 
        except pynmea2.ParseError:
            pass
        except Exception as e:
            rospy.logerr_throttle(5, f"GPS error: {e}")
 
    def run(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('ascii', errors='replace')
                self._process_line(line)
            except Exception as e:
                rospy.logerr_throttle(5, f"Serial read error: {e}")
 
    def shutdown(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")
 

if __name__ == '__main__':
    try:
        driver = LC29HDriver()
        rospy.on_shutdown(driver.shutdown)
        driver.run()
    except rospy.ROSInterruptException:
        pass