#!/usr/bin/env python3

"""
bluetooth_manager.py

Bluetooth RFCOMM transport implementing NetworkInterface.
Uses Linux AF_BLUETOOTH / BTPROTO_RFCOMM sockets — no third-party libraries required.

Because RFCOMM is a byte stream, each message is framed with a 4-byte big-endian
length prefix so the receiver can reconstruct packet boundaries.

Two modes controlled by the 'role' parameter:
  server  — bind to a local RFCOMM channel and wait for an incoming connection.
  client  — actively connect to a remote Bluetooth MAC address.
"""

import socket
import struct
import threading
import logging
from typing import Optional, Callable

from network_interface_base import NetworkInterface

_FRAME_HEADER = struct.Struct(">I")
_HEADER_SIZE = _FRAME_HEADER.size


class BluetoothDevice(NetworkInterface):
    """
    NetworkInterface over Bluetooth RFCOMM.

    Parameters
    ----------
    role : str
        'server' to accept connections, 'client' to initiate them.
    mac_address : str
        Remote MAC address (client mode) or '' to bind to any local adapter (server mode).
    channel : int
        RFCOMM channel number (1–30).
    logger : optional ROS2 or stdlib logger
    """

    def __init__(
        self,
        role: str,
        mac_address: str = "",
        channel: int = 1,
        logger=None,
    ):
        super().__init__(logger=logger)
        self._role = role.lower()
        self._mac = mac_address
        self._channel = channel

        self._server_sock: Optional[socket.socket] = None
        self._conn_sock: Optional[socket.socket] = None
        self._failed = False
        self._shutting_down = False

        self._rx_thread: Optional[threading.Thread] = None
        self._accept_thread: Optional[threading.Thread] = None

        self._write_lock = threading.Lock()

    def open(self) -> bool:
        self._failed = False
        self._shutting_down = False

        if self._role == "server":
            return self._open_server()
        elif self._role == "client":
            return self._open_client()
        else:
            self._log.error(f"Unknown Bluetooth role '{self._role}' — use 'server' or 'client'")
            self._failed = True
            return False

    def _open_server(self) -> bool:
        try:
            self._server_sock = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
            )
            self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_sock.bind(("00:00:00:00:00:00", self._channel))
            self._server_sock.listen(1)
            self._log.info(f"Bluetooth server listening on RFCOMM channel {self._channel}")

            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True, name="bt_accept"
            )
            self._accept_thread.start()
            return True
        except OSError as exc:
            self._log.error(f"Bluetooth server open failed: {exc}")
            self._failed = True
            return False

    def _accept_loop(self) -> None:
        while not self._shutting_down:
            try:
                self._server_sock.settimeout(2.0)
                conn, addr = self._server_sock.accept()
                self._log.info(f"Bluetooth client connected from {addr}")
                with self._write_lock:
                    if self._conn_sock:
                        try:
                            self._conn_sock.close()
                        except OSError:
                            pass
                    self._conn_sock = conn
                    self._failed = False

                rx = threading.Thread(
                    target=self._rx_loop,
                    args=(conn,),
                    daemon=True,
                    name="bt_rx",
                )
                rx.start()
                rx.join()
                self._log.info("Bluetooth client disconnected, waiting for new connection")
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._shutting_down:
                    self._log.error(f"Bluetooth accept error: {exc}")
                break

    def _open_client(self) -> bool:
        if not self._mac:
            self._log.error("Bluetooth client mode requires a mac_address")
            self._failed = True
            return False
        try:
            sock = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
            )
            sock.settimeout(10.0)
            sock.connect((self._mac, self._channel))
            sock.settimeout(None)
            with self._write_lock:
                self._conn_sock = sock
            self._log.info(f"Bluetooth connected to {self._mac} channel {self._channel}")

            self._rx_thread = threading.Thread(
                target=self._rx_loop,
                args=(sock,),
                daemon=True,
                name="bt_rx",
            )
            self._rx_thread.start()
            return True
        except OSError as exc:
            self._log.warning(f"Bluetooth client connect failed: {exc}")
            self._failed = True
            return False

    def _rx_loop(self, sock: socket.socket) -> None:
        try:
            while not self._shutting_down:
                header = self._recv_exactly(sock, _HEADER_SIZE)
                if header is None:
                    break
                (length,) = _FRAME_HEADER.unpack(header)
                if length == 0 or length > 2 * 1024 * 1024:
                    self._log.warning(f"Bluetooth: bad frame length {length}, dropping connection")
                    break
                payload = self._recv_exactly(sock, length)
                if payload is None:
                    break
                if self._receive_callback:
                    self._receive_callback(payload)
        except OSError as exc:
            if not self._shutting_down:
                self._log.error(f"Bluetooth RX error: {exc}")
        finally:
            with self._write_lock:
                if self._conn_sock is sock:
                    self._conn_sock = None
                    if self._role == "client":
                        self._failed = True
            try:
                sock.close()
            except OSError:
                pass

    def _recv_exactly(self, sock: socket.socket, n: int) -> Optional[bytes]:
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = sock.recv(n - len(buf))
            except OSError:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def close(self) -> None:
        self._shutting_down = True
        with self._write_lock:
            if self._conn_sock:
                try:
                    self._conn_sock.close()
                except OSError:
                    pass
                self._conn_sock = None
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None
        self._log.info("Bluetooth device closed.")

    def write(self, data: bytes) -> bool:
        with self._write_lock:
            sock = self._conn_sock
        if sock is None:
            self._log.debug("Bluetooth write: no active connection")
            return False
        try:
            frame = _FRAME_HEADER.pack(len(data)) + data
            sock.sendall(frame)
            return True
        except OSError as exc:
            self._log.error(f"Bluetooth write error: {exc}")
            with self._write_lock:
                if self._conn_sock is sock:
                    self._conn_sock = None
                    if self._role == "client":
                        self._failed = True
            try:
                sock.close()
            except OSError:
                pass
            return False

    def is_ready(self) -> bool:
        with self._write_lock:
            connected = self._conn_sock is not None
        if self._role == "server":
            return connected and not self._failed
        return connected and not self._failed

    def has_failed(self) -> bool:
        return self._failed
