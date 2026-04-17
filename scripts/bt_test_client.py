#!/usr/bin/env python3
import socket
import sys

MAC = sys.argv[1] if len(sys.argv) > 1 else "00:00:00:00:00:00"
CHANNEL = 1

print(f"Connecting to {MAC} channel {CHANNEL}...")
s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.settimeout(10.0)
s.connect((MAC, CHANNEL))
print("Connected.")
s.sendall(b"hello from client")
data = s.recv(64)
print(f"Received: {data}")
s.close()
print("Done.")
