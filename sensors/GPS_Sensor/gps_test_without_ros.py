import serial
import pynmea2

port = "/dev/ttyUSB0"
baud = 115200

try:
	ser = serial.Serial(port, baudrate=baud, timeout=20.5)
	print(f"Lese Daten von {port}")

	while True:
		line = ser.readline().decode('ascii', errors='replace')
		if line.startswith('$GNRMC'):
			try:
				msg = pynmea2.parse(line)
				if msg.status == 'A':
					print(f"Zeit: {msg.timestamp} | Breitengrad: {msg.latitude:.6f}  | Längengrad: {msg.longitude:.6f}")
				else:
					print("Warte auf GPS-Fix (Sichtkontakt zum Himmel?)...")
			except pynmea2.ParseError:
				continue

except KeyboardInterrupt:
	print("Beendet durch Nutzer.")
except Exception as e:
	print(f"Fehler: {e}")
