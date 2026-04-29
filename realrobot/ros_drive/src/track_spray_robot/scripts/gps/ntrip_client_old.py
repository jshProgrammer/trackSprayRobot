#!/usr/bin/env python3
import rospy
import socket
import base64
import time
import threading

"""
This node is responsible for RTK 
"""



class NTRIPClient:

    def __init__(self):

        # =========================
        # ROS PARAMS
        # =========================
        self.host = rospy.get_param("~host")
        self.port = rospy.get_param("~port", 2101)
        self.mountpoint = rospy.get_param("~mountpoint")
        self.username = rospy.get_param("~username")
        self.password = rospy.get_param("~password")

        self.reconnect_interval = 5.0 # rospy.get_param("~reconnect_interval", 5.0)

        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.serial_baud = rospy.get_param("~serial_baud", 115200)

        # =========================
        # SERIAL INIT
        # =========================
        try:
            import serial
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
            rospy.loginfo(f"GPS Serial verbunden: {self.serial_port}")
        except Exception as e:
            rospy.logerr(f"Serial Fehler: {e}")
            raise

        # =========================
        # THREAD START
        # =========================
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("NTRIP client started")

    # =========================
    # CONNECT TO CASTER
    # =========================
    def connect(self):

        auth = base64.b64encode(
            f"{self.username}:{self.password}".encode()
        ).decode()

        request = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP ROS Client\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        )

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((self.host, self.port))
            sock.send(request.encode())

            response = sock.recv(1024).decode(errors="ignore")

            if "200 OK" not in response:
                rospy.logerr(f"NTRIP connect failed: {response}")
                sock.close()
                return None

            rospy.loginfo("NTRIP verbunden")
            return sock

        except Exception as e:
            rospy.logerr(f"Connection error: {e}")
            return None

    # =========================
    # MAIN LOOP
    # =========================
    def run(self):

        sock = None

        while self.running and not rospy.is_shutdown():

            if sock is None:
                rospy.logwarn("Verbinde zu NTRIP...")
                sock = self.connect()

                if sock is None:
                    time.sleep(self.reconnect_interval)
                    continue

            try:
                start = time.time()
                data = sock.recv(4096)
                rospy.loginfo(f"RX bytes: {len(data)} in {time.time()-start:.2f}s")
                #data = sock.recv(4096)

                if not data:
                    raise ConnectionError("Leere Daten vom Caster")

                # RTCM direkt an GPS schicken
                self.ser.write(data)

            except socket.timeout:
                rospy.logwarn("NTRIP timeout (no data received)")
                continue
            #except Exception as e:
            #    rospy.logerr(f"NTRIP lost connection: {e}")
            #    sock.close()
            #    sock = None
            #    time.sleep(self.reconnect_interval)

    # =========================
    # SHUTDOWN
    # =========================
    def shutdown(self):
        rospy.loginfo("Shutting down NTRIP client...")
        self.running = False
        time.sleep(1)
        try:
            self.ser.close()
        except:
            pass


if __name__ == "__main__":
    rospy.init_node("ntrip_client_node")
    NTRIPClient()
    rospy.spin()