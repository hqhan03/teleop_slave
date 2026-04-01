import socket
import time

TARGET_IP = "192.168.0.112"
TARGET_PORT = 12345
MESSAGE = b"UDP Test Message from Windows"

print(f"Sending test packets to {TARGET_IP}:{TARGET_PORT}...")
print("Press Ctrl+C to stop.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        sock.sendto(MESSAGE, (TARGET_IP, TARGET_PORT))
        print(f"Sent: {MESSAGE.decode()} at {time.strftime('%H:%M:%S')}")
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
