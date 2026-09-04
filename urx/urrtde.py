"""
Minimal RTDE client, receive only.

Exists for one reason: the robot computes inverse kinematics, and the result has to
get back to us. Sending it over a socket the robot opens means we must be addressable
from the robot, which is a guess behind any NAT. RTDE is a connection we open, so the
result travels back over a path we already own.

Only what that needs: negotiate, subscribe to some output variables, read frames. No
input registers, no control script, so this composes with the program sending on the
secondary interface rather than competing with it.

Protocol reference: https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
"""

import logging
import socket
import struct


RTDE_PORT = 30004
PROTOCOL_VERSION = 2

# Package types, as ASCII codes.
_REQUEST_PROTOCOL_VERSION = 86  # V
_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # O
_CONTROL_PACKAGE_START = 83  # S
_DATA_PACKAGE = 85  # U
_TEXT_MESSAGE = 77  # M

_HEADER = struct.Struct(">HB")

# Only the types this client subscribes to. Anything else is a programming error here,
# not a runtime condition to absorb.
_FORMATS = {
    "INT32": struct.Struct(">i"),
    "DOUBLE": struct.Struct(">d"),
}

# 125Hz is the CB3 control rate and a valid divisor of the e-series 500Hz, so one value
# works on both.
DEFAULT_FREQUENCY = 125.0


class RTDEError(Exception):
    pass


class RTDEClient(object):
    """Subscribes to named output variables and hands back one frame at a time."""

    def __init__(self, host, variables, frequency=DEFAULT_FREQUENCY, logger=None):
        self.host = host
        self.variables = list(variables)
        self.frequency = frequency
        self.logger = logger or logging.getLogger("URX Logger")
        self._sock = None
        self._recipe_id = None
        self._layout = None
        self._buf = b""

    # ---------- framing ----------

    def _send(self, package_type, payload=b""):
        size = _HEADER.size + len(payload)
        self._sock.sendall(_HEADER.pack(size, package_type) + payload)

    def _recv_package(self):
        """The next package, as (type, payload). Blocks until one is complete."""
        while True:
            while len(self._buf) < _HEADER.size:
                self._fill()
            size, package_type = _HEADER.unpack(self._buf[: _HEADER.size])
            if size < _HEADER.size:
                raise RTDEError(f"RTDE framing error: package size {size}")
            while len(self._buf) < size:
                self._fill()
            payload = self._buf[_HEADER.size : size]
            self._buf = self._buf[size:]

            # The controller volunteers these; they are not replies to anything, so
            # returning one to a caller waiting for a reply would desynchronise us.
            if package_type == _TEXT_MESSAGE:
                self.logger.warning("RTDE text message: %s", payload)
                continue
            return package_type, payload

    def _fill(self):
        chunk = self._sock.recv(4096)
        if not chunk:
            raise RTDEError("RTDE connection closed by the controller")
        self._buf += chunk

    def _request(self, package_type, payload=b""):
        """Send and read the matching reply, which the protocol tags with the same type."""
        self._send(package_type, payload)
        reply_type, reply = self._recv_package()
        if reply_type != package_type:
            raise RTDEError(
                f"RTDE expected a {package_type} reply, got {reply_type}"
            )
        return reply

    # ---------- lifecycle ----------

    def connect(self, timeout=5.0):
        self._sock = socket.create_connection((self.host, RTDE_PORT), timeout=timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = b""

        reply = self._request(
            _REQUEST_PROTOCOL_VERSION, struct.pack(">H", PROTOCOL_VERSION)
        )
        if not struct.unpack(">B", reply)[0]:
            raise RTDEError(f"Controller refused RTDE protocol {PROTOCOL_VERSION}")

        payload = struct.pack(">d", self.frequency) + ",".join(self.variables).encode()
        reply = self._request(_CONTROL_PACKAGE_SETUP_OUTPUTS, payload)
        self._recipe_id = reply[0]
        types = reply[1:].decode().split(",")
        if len(types) != len(self.variables):
            raise RTDEError(f"RTDE returned {len(types)} types for {len(self.variables)} variables")
        unknown = [
            name for name, kind in zip(self.variables, types) if kind not in _FORMATS
        ]
        if unknown:
            raise RTDEError(f"Controller does not provide these RTDE outputs: {unknown}")
        self._layout = list(zip(self.variables, [_FORMATS[kind] for kind in types]))

        if not struct.unpack(">B", self._request(_CONTROL_PACKAGE_START))[0]:
            raise RTDEError("Controller refused to start the RTDE stream")

        self.logger.debug(
            "RTDE connected to %s, recipe %s, %s variables",
            self.host, self._recipe_id, len(self.variables),
        )

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._layout = None

    # ---------- reading ----------

    def read(self, timeout=None):
        """The next frame, as {variable_name: value}."""
        if timeout is not None:
            self._sock.settimeout(timeout)
        while True:
            package_type, payload = self._recv_package()
            if package_type != _DATA_PACKAGE:
                self.logger.debug("RTDE ignoring package type %s", package_type)
                continue
            if payload[0] != self._recipe_id:
                self.logger.debug("RTDE ignoring recipe %s", payload[0])
                continue

            frame = {}
            offset = 1
            for name, fmt in self._layout:
                (frame[name],) = fmt.unpack_from(payload, offset)
                offset += fmt.size
            return frame
