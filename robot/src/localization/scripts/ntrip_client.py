#!/usr/bin/env python3
"""
NTRIP Client as ROS-Node
- Receives GGA via Topic  gps/nmea_sentence  (no direct Serial-access, sent by gps_node)
- Sends RTCM Corrections via Topic gps/rtcm  (written to Serial by gps_node)

Fix: AF_UNSPEC statt AF_INET – Router blockiert IPv4 Port 2101, IPv6 funktioniert.
"""

import socket
import base64
import datetime
import time
import threading
import os
import logging
from typing import Optional

import rospy
from std_msgs.msg import String, UInt8MultiArray
from robot_msgs.status import StatusReporter

version    = 0.2
useragent  = "NTRIP JCMBsoftPythonClient/%.1f" % version

FACTOR             = 2
MAX_RECONNECT_TIME = 30
INITIAL_SLEEP      = 1
GGA_INTERVAL       = 5.0
SOCKET_TIMEOUT     = 2.0
CONNECT_TIMEOUT    = 10.0


class NtripClientNode:
    def __init__(self):
        rospy.init_node('ntrip_client', anonymous=True)

        self.caster     = rospy.get_param("~caster",     "")
        self.port       = rospy.get_param("~port",       2101)
        self.mountpoint = rospy.get_param("~mountpoint", "")
        self.user       = rospy.get_param("~user",       "")
        self.password   = rospy.get_param("~password",   "")
        self.verbose    = rospy.get_param("~verbose",    False)

        user_pass  = f"{self.user}:{self.password}"
        self.auth  = base64.b64encode(user_pass.encode()).decode()

        self.latest_gga = None
        self.gga_lock   = threading.Lock()

        self.rtcm_pub = rospy.Publisher('gps/rtcm', UInt8MultiArray, queue_size=10)
        rospy.Subscriber('gps/nmea_sentence', String, self._nmea_callback)

        self.status = StatusReporter(source="ntrip_client")
        self.init_logging()

        rospy.loginfo("NTRIP Client Node gestartet, wartet auf ersten GGA-Satz...")
        self.logger.info("NTRIP Client Node gestartet, wartet auf ersten GGA-Satz...")

    # =========================
    # Logging
    # =========================
    def init_logging(self):
        log_dir = os.path.expanduser(
            f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, "ntrip_client.log")

        self.logger = logging.getLogger("ntrip_client")
        self.logger.setLevel(logging.INFO)
        handler   = logging.FileHandler(log_file)
        handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
        self.logger.addHandler(handler)

    # =========================
    # NMEA Callback
    # =========================
    def _nmea_callback(self, msg: String):
        line = msg.data.strip()
        self.logger.info(f"NMEA empfangen: {line[:20]}")
        if 'GGA' in line:
            with self.gga_lock:
                self.latest_gga = line

    def _get_latest_gga_bytes(self) -> Optional[bytes]:
        with self.gga_lock:
            gga = self.latest_gga
        if gga:
            self.logger.info(f"Letzte GGA: {gga}")
            return (gga + "\r\n").encode('ascii')
        return None

    def _wait_for_gga(self, timeout=60.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._get_latest_gga_bytes():
                return True
            rospy.sleep(0.2)
        return False

    # =========================
    # Connection helpers
    # =========================
    def _resolve_addr(self):
        """
        Löst den Hostnamen mit AF_UNSPEC auf – nimmt was funktioniert.
        IPv4 Port 2101 wird vom Router blockiert, IPv6 geht durch.
        Bevorzugt IPv6 wenn verfügbar.
        """
        try:
            results = socket.getaddrinfo(
                self.caster, self.port,
                socket.AF_UNSPEC, socket.SOCK_STREAM
            )
            self.logger.info(f"DNS-Ergebnisse: {results}")

            # IPv6 bevorzugen (IPv4 wird vom Router geblockt)
            for r in results:
                if r[0] == socket.AF_INET6:
                    self.logger.info(f"Verwende IPv6: {r[4]}")
                    return r[0], r[4]

            # Fallback IPv4
            r = results[0]
            self.logger.info(f"Verwende IPv4 (Fallback): {r[4]}")
            return r[0], r[4]

        except socket.gaierror as e:
            self.logger.error(f"DNS-Aufloesung fehlgeschlagen: {e}")
            self.status.warn("NTRIP_DNS_FAILED", "NTRIP-Server nicht auflösbar (DNS)")
            return None, None

    def _build_request(self) -> bytes:
        mountpoint = "/" + self.mountpoint
        req = (
            f"GET {mountpoint} HTTP/1.1\r\n"
            f"Host: {self.caster}:{self.port}\r\n"
            f"User-Agent: {useragent}\r\n"
            f"Authorization: Basic {self.auth}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        )
        self.logger.info(f"Request:\n{req}")
        if self.verbose:
            rospy.loginfo(f"Request:\n{req}")
        return req.encode('ascii')

    def _parse_header(self, raw: bytes):
        text = raw.decode('utf-8', errors='replace')
        if "SOURCETABLE" in text:
            return False, "Mountpoint existiert nicht (SOURCETABLE erhalten)"
        if "401" in text:
            return False, f"Unauthorized (401) – Zugangsdaten prüfen"
        if "403" in text:
            return False, f"Forbidden (403)"
        if "404" in text:
            return False, f"Mountpoint nicht gefunden (404)"
        if any(ok in text for ok in ("ICY 200 OK", "HTTP/1.0 200 OK", "HTTP/1.1 200 OK")):
            return True, ""
        return None, ""

    def _send_gga(self, sock: socket.socket) -> bool:
        gga = self._get_latest_gga_bytes()
        if not gga:
            return True
        try:
            sock.sendall(gga)
            self.logger.info(f"GGA gesendet: {gga.strip()}")
            return True
        except socket.error as e:
            self.logger.warning(f"GGA senden fehlgeschlagen: {e}")
            return False

    def _publish_rtcm(self, data: bytes):
        msg      = UInt8MultiArray()
        msg.data = list(data)
        self.rtcm_pub.publish(msg)
        self.logger.info(f"RTCM {len(data)} Bytes publiziert")

    # =========================
    # Main loop
    # =========================
    def run(self):
        if not self._wait_for_gga(timeout=60):
            msg = "Kein GGA-Satz empfangen – NTRIP Client beendet sich."
            rospy.logerr(msg)
            self.logger.error(msg)
            self.status.report_fatal("NTRIP_NO_GGA", "Kein GGA empfangen – NTRIP beendet")
            return

        sleep_time = INITIAL_SLEEP

        while not rospy.is_shutdown():
            # ── Adresse auflösen (IPv6 bevorzugt) ───────────────────────
            family, addr = self._resolve_addr()
            if addr is None:
                rospy.sleep(sleep_time)
                sleep_time = min(sleep_time * FACTOR, MAX_RECONNECT_TIME)
                continue

            msg = f"Verbinde mit {self.caster}:{self.port}/{self.mountpoint} ({addr[0]})"
            rospy.loginfo(msg)
            self.logger.info(msg)

            sock = socket.socket(family, socket.SOCK_STREAM)
            sock.settimeout(CONNECT_TIMEOUT)

            # ── Connect ─────────────────────────────────────────────────
            try:
                sock.connect(addr)
            except socket.error as e:
                msg = f"Verbindungsfehler: {e}. Retry in {sleep_time}s"
                rospy.logwarn(msg)
                self.logger.warning(msg)
                self.status.warn("NTRIP_DISCONNECTED", "NTRIP-Verbindungsfehler – Reconnect")
                sock.close()
                rospy.sleep(sleep_time)
                sleep_time = min(sleep_time * FACTOR, MAX_RECONNECT_TIME)
                continue

            try:
                # 1. HTTP-Request senden
                sock.sendall(self._build_request())

                # 2. GGA direkt nach Request senden (SAPOS erwartet das für VRS)
                self._send_gga(sock)

                # 3. Header lesen & prüfen
                sock.settimeout(CONNECT_TIMEOUT)
                header_raw = b""
                ok = None
                while ok is None:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    header_raw += chunk
                    ok, err = self._parse_header(header_raw)
                    if ok is False:
                        rospy.logerr(err)
                        self.logger.error(err)
                        self.status.report_fatal("NTRIP_AUTH_FAILED",
                                                 f"NTRIP abgelehnt: {err}")
                        return  # fataler Fehler

                if not ok:
                    rospy.logwarn("Unvollständiger Header, Reconnect...")
                    self.logger.warning(f"Unvollständiger Header: {repr(header_raw[:200])}")
                    continue

                # 4. Nochmal GGA senden nach Header-OK
                self._send_gga(sock)

                rospy.loginfo("NTRIP verbunden – empfange RTCM-Korrekturen")
                self.logger.info("NTRIP verbunden – empfange RTCM-Korrekturen")
                self.status.info("NTRIP_CONNECTED", "NTRIP verbunden – empfange Korrekturen")

                # Verbindung war erfolgreich → Backoff zurücksetzen
                sleep_time    = INITIAL_SLEEP
                last_gga_time = time.time()

                sock.settimeout(SOCKET_TIMEOUT)

                # 5. RTCM-Schleife
                while not rospy.is_shutdown():
                    try:
                        data = sock.recv(1024)
                        if not data:
                            rospy.logwarn("Verbindung vom Caster getrennt.")
                            self.logger.warning("Verbindung vom Caster getrennt.")
                            self.status.warn("NTRIP_DISCONNECTED", "Caster getrennt – Reconnect")
                            break
                        self._publish_rtcm(data)

                    except socket.timeout:
                        pass  # normal, weiter

                    # GGA periodisch senden
                    if time.time() - last_gga_time >= GGA_INTERVAL:
                        if not self._send_gga(sock):
                            break
                        last_gga_time = time.time()

            except socket.error as e:
                msg = f"Verbindungsfehler: {e}. Retry in {sleep_time}s"
                rospy.logwarn(msg)
                self.logger.warning(msg)
                self.status.warn("NTRIP_DISCONNECTED", "NTRIP-Verbindungsfehler – Reconnect")
            finally:
                sock.close()

            rospy.logwarn(f"Getrennt. Reconnect in {sleep_time}s")
            self.logger.warning(f"Getrennt. Reconnect in {sleep_time}s")
            rospy.sleep(sleep_time)
            sleep_time = min(sleep_time * FACTOR, MAX_RECONNECT_TIME)


# ----------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = NtripClientNode()
        node.run()
    except rospy.ROSInterruptException:
        pass