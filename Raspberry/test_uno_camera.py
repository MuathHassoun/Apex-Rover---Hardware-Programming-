import serial
import time

UNO_PORT = "/dev/ttyACM0"   # change if needed
BAUD = 9600

uno = serial.Serial(UNO_PORT, BAUD, timeout=1)
time.sleep(2)

def send(cmd):
    print("SEND:", cmd)
    uno.write((cmd + "\n").encode())
    time.sleep(0.3)

    while uno.in_waiting:
        print("UNO:", uno.readline().decode(errors="ignore").strip())

send("CAM:STATUS")

send("CAM:RIGHT")
time.sleep(2)
send("CAM:STOP")

send("CAM:LEFT")
time.sleep(2)
send("CAM:STOP")

send("CAM:UP")
send("CAM:DOWN")
send("CAM:CENTER")

uno.close()
