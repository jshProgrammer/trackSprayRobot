#!/usr/bin/env python3
import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8MultiArray, String, UInt8
import datetime
import os
import logging

class LC29HDriver:
    def __init__(self):
        rospy.init_node('lc29h_driver', anonymous=True)
 
        self.init_logging()
        self._init_params()
        self._init_serial()
        self._init_publishers()
        self._init_subscribers()
 
    # =========================
    # Receiving parameters from config
    # =========================
    def _init_params(self):
        self.port       = rospy.get_param("~port",       "/dev/ttyUSB0")
        self.baud       = rospy.get_param("~baud",       115200)
        self.frame_id   = rospy.get_param("~frame_id",   "gps_link")
        self.gps_rate_hz = rospy.get_param("~gps_rate_hz", 5)
 
    def _init_serial(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.5)
            debugOutput = f"LC29H connected on {self.port}"
            self.logger.info(debugOutput)
            rospy.loginfo(debugOutput)
            self._configure_rate(self.gps_rate_hz)
        except Exception as e:
            debugOutput = f"Serial error: {e}"
            self.logger.error(debugOutput)
            rospy.logerr(debugOutput)
            raise

    def _nmea_checksum(self, sentence: str) -> str:
        checksum = 0
        for ch in sentence:
            checksum ^= ord(ch)
        return f"{checksum:02X}"

    def _configure_rate(self, rate_hz: int):
        rate_hz  = max(1, min(10, rate_hz))
        interval = 1000 // rate_hz          # ms
        body     = f"PMTK220,{interval}"
        cmd      = f"${body}*{self._nmea_checksum(body)}\r\n"
        self.ser.write(cmd.encode('ascii'))
        debugOutput = f"GPS update rate set to {rate_hz} Hz (interval={interval}ms): {cmd.strip()}"
        self.logger.info(debugOutput)
        rospy.loginfo(debugOutput)

    # =========================
    # Publisher initialization
    # =========================
    def _init_publishers(self):
        #TODO: probably only use gps in the futures (with fixed and unfixed together), and then just filter
        self.pub         = rospy.Publisher('gps/fix',           NavSatFix, queue_size=10)
        # unfixed publisher will probably be removed in the future!
        self.unfixed_pub = rospy.Publisher('gps/unfixed',       NavSatFix, queue_size=10)
        self.nmea_pub    = rospy.Publisher('gps/nmea_sentence', String,    queue_size=10)
        self.qual_pub = rospy.Publisher(
            'gps/quality',
            UInt8,
            queue_size=10
        )
 
    def _init_subscribers(self):
        rospy.Subscriber('gps/rtcm', UInt8MultiArray, self._rtcm_callback)

    # =========================
    # Additional Debug logging
    # =========================
    def init_logging(self):
        log_dir = os.path.expanduser(f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(log_dir, exist_ok=True)

        log_file = os.path.join(
            log_dir,
            "gps_node.log"
        )

        self.logger = logging.getLogger("gps_node")
        self.logger.setLevel(logging.INFO)

        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s'
        )

        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

 
    # =========================
    # RTCM Callback (for NTRIP Client / RTK)
    # =========================
    def _rtcm_callback(self, msg):
        try:
            self.ser.write(bytes(msg.data))
        except Exception as e:
            debugOutput = f"RTCM write error: {e}"
            rospy.logerr_throttle(5, debugOutput)
            self.logger.error(debugOutput)

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

        # GPS QUALITY (RTK STATUS)
        if gps_qual == 4:
            fix.status.status = NavSatStatus.STATUS_FIX
            rospy.loginfo_throttle(1, "RTK FIX (cm-level)")
        elif gps_qual == 5:
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            rospy.logwarn_throttle(1, "RTK FLOAT")
        elif gps_qual == 2:
            fix.status.status = NavSatStatus.STATUS_FIX
            rospy.loginfo_throttle(5, "DGPS FIX")
        elif gps_qual == 1:
            fix.status.status = NavSatStatus.STATUS_FIX
            rospy.loginfo_throttle(5, "GPS FIX")
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            rospy.logwarn_throttle(5, "No GPS Fix")
            return None

        fix.latitude  = float(msg.latitude)
        fix.longitude = float(msg.longitude)
        fix.altitude  = float(msg.altitude) if msg.altitude else 0.0

        fix.position_covariance = [0.0] * 9
        if gps_qual == 4:
            fix.position_covariance[0] = 0.01  # RTK FIX: ~cm precision
        elif gps_qual == 5:
            fix.position_covariance[0] = 0.1   # RTK FLOAT
        elif gps_qual == 2:
            fix.position_covariance[0] = 1.0   # DGPS
        else:
            fix.position_covariance[0] = 5.0   # GPS

        rospy.loginfo(f"GPS Qual: {gps_qual}")

        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        return fix
 
    def _process_line(self, line: str):
        # pass all NMEA sentences for NTRIP_Client
        if line.startswith('$'):
            self.logger.info(f"NMEA: {line.strip()}")
            self.nmea_pub.publish(String(data=line))
 
        if not line.startswith('$GNGGA'):
            return
 
        try:
            msg      = pynmea2.parse(line)
            gps_qual = int(msg.gps_qual)

            self.qual_pub.publish(UInt8(data=gps_qual))

            self.logger.info(f"Parsed GGA - Lat: {msg.latitude}, Lon: {msg.longitude}, Alt: {msg.altitude}, Qual: {gps_qual}")
 
            self.unfixed_pub.publish(self._build_unfixed(msg))
 
            # =========================
            # PUBLISH
            # =========================
            fix = self._build_fix(msg, gps_qual)
            if fix is not None:
                self.logger.info(f"Publishing GPS Fix - Lat: {fix.latitude}, Lon: {fix.longitude}, Alt: {fix.altitude}, Status: {fix.status.status}")
                self.pub.publish(fix)
 
        except pynmea2.ParseError:
            pass
        except Exception as e:
            debugOutput = f"GPS error: {e}"
            self.logger.error(debugOutput)
            rospy.logerr_throttle(5, debugOutput)
 
    def run(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('ascii', errors='replace')
                self._process_line(line)
            except Exception as e:
                debugOutput = f"Serial read error: {e}"
                self.logger.error(debugOutput)
                rospy.logerr_throttle(5, debugOutput)
 
    def shutdown(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            debugOutput = "Serial port closed"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)
 

if __name__ == '__main__':
    try:
        driver = LC29HDriver()
        rospy.on_shutdown(driver.shutdown)
        driver.run()
    except rospy.ROSInterruptException:
        pass