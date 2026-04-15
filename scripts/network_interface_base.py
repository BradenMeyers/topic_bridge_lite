#!/usr/bin/env python3

"""
network_interface_base.py

Abstract base class for Python transport implementations.
Mirrors the C++ NetworkInterface in include/network_interfaces/network_interface_base.hpp.

Implement open(), close(), write(), is_ready(), and has_failed().
Call self._receive_callback(data: bytes) when bytes arrive from the hardware.
"""

import logging
from abc import ABC, abstractmethod
from typing import Callable, Optional


class NetworkInterface(ABC):

    def __init__(self, logger=None):
        self._log = logger or logging.getLogger(__name__)
        self._receive_callback: Optional[Callable[[bytes], None]] = None

    def set_receive_callback(self, fn: Callable[[bytes], None]) -> None:
        self._receive_callback = fn

    @abstractmethod
    def open(self) -> bool:
        """Open the hardware connection. Return True on success."""

    @abstractmethod
    def close(self) -> None:
        """Close the hardware connection cleanly."""

    @abstractmethod
    def write(self, data: bytes) -> bool:
        """Send bytes to the remote. Return True on success."""

    @abstractmethod
    def is_ready(self) -> bool:
        """Return True if the connection is open and healthy."""

    @abstractmethod
    def has_failed(self) -> bool:
        """Return True if the connection needs to be reopened."""
