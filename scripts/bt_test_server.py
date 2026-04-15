#!/usr/bin/env python3
import socket

CHANNEL = 1

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("00:00:00:00:00:00", CHANNEL))
s.listen(1)
print(f"Listening on RFCOMM channel {CHANNEL} — waiting for client...")
conn, addr = s.accept()
print(f"Connected from {addr}")
conn.sendall(b"hello from server")
data = conn.recv(64)
print(f"Received: {data}")
conn.close()
s.close()
print("Done.")
