#!/usr/bin/env python3

"""
radio_manager.py

XBee hardware wrapper implementing NetworkInterface.
Owns the serial port only — no framing, no routing, no ACKs.
Bytes go in, bytes come out, unchanged.
"""

import logging
from typing import Optional

from network_interface_base import NetworkInterface

try:
    from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
    from digi.xbee.exception import TransmitException, TimeoutException
    _XBEE_AVAILABLE = True
except ImportError:
    _XBEE_AVAILABLE = False


class XBeeRadioDevice(NetworkInterface):
    """
    NetworkInterface for digi XBee radios.

    XBee handles checksums and destination routing at the RF layer so this
    class just passes raw bytes through. Unicast to dest_address if set,
    broadcast otherwise.
    """

    def __init__(self, port: str, baud: int, dest_address: Optional[str] = None, logger=None):
        super().__init__(logger=logger)
        self._port = port
        self._baud = baud
        self._dest_address = dest_address
        self._device = None
        self._failed = False

        if not _XBEE_AVAILABLE:
            self._log.warning("digi.xbee not installed — XBeeRadioDevice in stub mode.")

    def open(self) -> bool:
        if not _XBEE_AVAILABLE:
            self._log.warning("XBee stub: open() called, no hardware.")
            return False
        try:
            self._device = XBeeDevice(self._port, self._baud)
            self._device.open()
            self._device.add_data_received_callback(self._hw_rx_callback)
            self._failed = False
            self._log.info(f"XBee opened on {self._port} @ {self._baud} baud.")
            return True
        except Exception as exc:
            self._log.error(f"XBee open failed: {exc}")
            self._failed = True
            return False

    def close(self) -> None:
        if self._device and self._device.is_open():
            self._device.close()
            self._log.info("XBee device closed.")
        self._device = None

    def write(self, data: bytes) -> bool:
        if not _XBEE_AVAILABLE or self._device is None:
            self._log.debug(f"XBee stub: send {len(data)} bytes to {self._dest_address or 'broadcast'}")
            return True
        try:
            if self._dest_address:
                remote = RemoteXBeeDevice(
                    self._device, XBee64BitAddress.from_hex_string(self._dest_address)
                )
                self._device.send_data(remote, data)
            else:
                self._device.send_data_broadcast(data)
            return True
        except (TransmitException, TimeoutException) as exc:
            if self._dest_address:
                self._log.error(f"XBee send error (unicast to {self._dest_address}): {exc}")
                return False
            # Broadcast with no receivers in range is not a real failure.
            self._log.debug(f"XBee broadcast send error (no receivers): {exc}")
            return True
        except Exception as exc:
            self._log.error(f"XBee send error: {exc}")
            self._failed = True
            return False

    def is_ready(self) -> bool:
        return self._device is not None and self._device.is_open() and not self._failed

    def has_failed(self) -> bool:
        return self._failed

    def _hw_rx_callback(self, xbee_message) -> None:
        if self._receive_callback:
            self._receive_callback(bytes(xbee_message.data))
