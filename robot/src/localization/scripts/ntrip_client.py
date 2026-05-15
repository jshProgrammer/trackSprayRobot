#!/usr/bin/env python3
"""
NTRIP Client as ROS-Node
- Receives GGA via Topic  gps/nmea_sentence  (no direct Serial-access, sent by gps_node)
- Sends RTCM Corrections via Topic gps/rtcm  (written to Serial by gps_node)
"""

import socket
import base64
import datetime
import time
import threading
import os
import logging

import rospy
from std_msgs.msg import String, UInt8MultiArray

version    = 0.2
useragent  = "NTRIP JCMBsoftPythonClient/%.1f" % version

# Reconnect-Parameter
FACTOR           = 2
MAX_RECONNECT    = 10
MAX_RECONNECT_TIME = 1200
INITIAL_SLEEP    = 1


class NtripClientNode:
    def __init__(self):
        rospy.init_node('ntrip_client', anonymous=True)

        # =========================
        # Receiving parameters from config
        # =========================
        self.caster     = rospy.get_param("~caster",     "")
        self.port       = rospy.get_param("~port",       2101)
        self.mountpoint = rospy.get_param("~mountpoint", "")
        self.user       = rospy.get_param("~user",       "")
        self.password   = rospy.get_param("~password",   "")
        self.verbose    = rospy.get_param("~verbose",    False)

        user_pass = f"{self.user}:{self.password}"
        self.auth = base64.b64encode(user_pass.encode()).decode()

        # Last know GGA
        self.latest_gga = None
        self.gga_lock   = threading.Lock()

        # =========================
        # Setting up subscriber and receiver
        # =========================
        self.rtcm_pub = rospy.Publisher(
            'gps/rtcm', UInt8MultiArray, queue_size=10
        ) # sending to gps node

        rospy.Subscriber('gps/nmea_sentence', String, self._nmea_callback) # receiving from gps node

        # Debug logging
        self.init_logging()

        debugOutput = "NTRIP Client Node gestartet, wartet auf ersten GGA-Satz..."
        rospy.loginfo(debugOutput)
        self.logger.info(debugOutput)

    # =========================
    # Additional Debug logging
    # =========================
    def init_logging(self):
        log_dir = os.path.expanduser(f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(log_dir, exist_ok=True)

        log_file = os.path.join(
            log_dir,
            "ntrip_client.log"
        )

        self.logger = logging.getLogger("ntrip_client")
        self.logger.setLevel(logging.INFO)

        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s'
        )

        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    # =========================
    # NMEA Callback
    # =========================
    def _nmea_callback(self, msg: String):
        """Stores the current GGA-value thread-safe."""
        line = msg.data.strip()
        self.logger.info(f"NMEA empfangen: {line[:20]}")
        if 'GGA' in line:
            with self.gga_lock:
                self.latest_gga = line

    def _get_latest_gga_bytes(self) -> bytes:
        with self.gga_lock:
            gga = self.latest_gga
        if gga:
            self.logger.info(f"Letzte GGA Bytes: {gga}")
            return (gga + "\r\n").encode('ascii')
        return None

    # ------------------------------------------------------------------
    def _build_request(self) -> bytes:
        mountpoint = "/" + self.mountpoint
        req = (
            f"GET {mountpoint} HTTP/1.1\r\n"
            f"User-Agent: {useragent}\r\n"
            f"Authorization: Basic {self.auth}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"\r\n"
        )

        debugOutput = f"Request:\n{req}"
        
        if self.verbose:
            rospy.loginfo(debugOutput)
        
        self.logger.info(debugOutput)

        return req.encode('ascii')

    # ------------------------------------------------------------------
    def _wait_for_gga(self, timeout=30.0) -> bool:
        """Blocks until GGA-value is present or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._get_latest_gga_bytes():
                return True
            rospy.sleep(0.2)
        return False

    # ------------------------------------------------------------------
    def _parse_header(self, raw: bytes):
        """
        Returns (ok, error_msg)
        ok=True  → Connection accepted, GGA sent
        ok=False → fatal Error, terminated node
        """
        text = raw.decode('utf-8', errors='replace')
        if "SOURCETABLE" in text:
            return False, "Mountpoint existiert nicht (SOURCETABLE erhalten)"
        if "401 Unauthorized" in text:
            return False, "Unauthorized – Zugangsdaten prüfen"
        if "404 Not Found" in text:
            return False, "Mountpoint nicht gefunden (404)"
        if any(ok in text for ok in ("ICY 200 OK", "HTTP/1.0 200 OK", "HTTP/1.1 200 OK")):
            return True, ""
        # Noch nicht vollständig – weiterlesen
        return None, ""

    # ------------------------------------------------------------------
    def _publish_rtcm(self, data: bytes):
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.logger.info(f"RTCM bytes werden gepublished: {data}")
        self.rtcm_pub.publish(msg)

    # ------------------------------------------------------------------
    def run(self):
        # Wait until first GGA received
        if not self._wait_for_gga(timeout=60):
            debugOutput = "Kein GGA-Satz empfangen – NTRIP Client beendet sich."
            rospy.logerr(debugOutput)
            self.logger.info(debugOutput)
            return

        debugOutput = f"Verbinde mit {self.caster}:{self.port}/{self.mountpoint}"
        self.logger.info(debugOutput)

        sleep_time    = INITIAL_SLEEP
        reconnect_try = 0

        while not rospy.is_shutdown():
            reconnect_try += 1
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #sock.settimeout(10)

            try:
                sock.connect((self.caster, self.port))
                sock.settimeout(10.0)
            except socket.error as e:
                debugOutput = f"Verbindungsfehler: {e}. Retry in {sleep_time}s"
                rospy.logwarn(debugOutput)
                self.logger.warning(debugOutput)
                sock.close()
                rospy.sleep(sleep_time)
                sleep_time = min(sleep_time * FACTOR, MAX_RECONNECT_TIME)
                continue

            try:
                # 1. send request
                sock.sendall(self._build_request())

                # 2. receive header and validate
                header_raw = b""
                ok = None
                while ok is None:
                    header_raw += sock.recv(1024)
                    ok, err = self._parse_header(header_raw)
                    if ok is False:
                        rospy.logerr(err)
                        self.logger.error(err)
                        return  # fatal Error

                # 3. send GGA
                gga = self._get_latest_gga_bytes()
                if gga:
                    sock.sendall(gga)
                    debugOutput = f"GGA gesendet: {gga.strip()}"
                    self.logger.info(debugOutput)

                # 4. read RTCM-data
                debugOutput = "NTRIP verbunden – empfange RTCM-Korrekturen"
                rospy.loginfo(debugOutput)
                self.logger.info(debugOutput)

                sock.settimeout(5.0)
                sleep_time    = INITIAL_SLEEP  # reset after success
                reconnect_try = 0

                last_gga_time = time.time()
                GGA_INTERVAL  = 1.0  # seconds between GGA updates

                while not rospy.is_shutdown():
                    try:
                        #TODO: if not working change back to 4096
                        data = sock.recv(1024)
                        if not data:
                            debugOutput = "Verbindung vom Caster getrennt."
                            rospy.logwarn(debugOutput)
                            self.logger.warning(debugOutput)
                            break

                        self._publish_rtcm(data)

                        # send new GGA periodically
                        if time.time() - last_gga_time > GGA_INTERVAL:
                            gga = self._get_latest_gga_bytes()
                            if gga:
                                sock.sendall(gga)
                            last_gga_time = time.time()

                    except socket.timeout:
                        debugOutput = "Socket Timeout – warte auf Daten"
                        rospy.logwarn_throttle(10, debugOutput)
                        self.logger.warning(debugOutput)
                    except socket.error as e:
                        debugOutput = f"Socket Fehler: {e}"
                        rospy.logwarn(debugOutput)
                        self.logger.warning(debugOutput)
                        break

            finally:
                sock.close()

            debugOutput = f"Getrennt. Reconnect in {sleep_time}s"
            rospy.logwarn(debugOutput)
            self.logger.warning(debugOutput)

            rospy.sleep(sleep_time)
            sleep_time = min(sleep_time * FACTOR, MAX_RECONNECT_TIME)


# ----------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = NtripClientNode()
        node.run()
    except rospy.ROSInterruptException:
        pass