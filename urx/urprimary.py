"""
Module for reading robot messages from the primary interface on port 30001.

Runtime exceptions, popups, error codes and text messages reach primary clients only -
the secondary interface carries robot state and nothing else. A fault that stops a
running program therefore leaves no trace on 30002, which is why a move can fail with
nothing but "program stopped" to go on.
"""

import logging
import socket
import struct
import time
from collections import deque
from threading import Lock, Thread

from urx.ursecmon import ParserUtils, ParsingException, printable_text

PRIMARY_PORT = 30001
HEADER = struct.Struct("!iB")
MAX_PACKET_SIZE = 2**21
RECONNECT_DELAY = 2.0

# Keys ParserUtils produces for the type-20 messages worth reporting, in the order
# they are checked. Everything else is state.
MESSAGE_KEYS = (
    "runtimeExceptionMessage",
    "popupMessage",
    "robotCommMessage",
    "safetyModeMessage",
    "messageText",
    "keyMessage",
    "requestValueMessage",
)


def _text(message, key="messageText"):
    raw = message.get(key, b"")
    return raw.decode("utf-8", errors="replace").strip()


def _describe(key, message):
    if key == "runtimeExceptionMessage":
        return "runtime exception at script line {}, column {}: {}".format(
            message["scriptLineNumber"], message["scriptColumnNumber"], _text(message)
        )
    if key == "popupMessage":
        kind = "error" if message["error"] else "warning" if message["warning"] else "popup"
        return "{}: {} - {}".format(
            kind, _text(message, "messageTitle"), _text(message)
        )
    if key == "robotCommMessage":
        return "error code {}.{}: {}".format(
            message["code"], message["argument"], _text(message)
        )
    if key == "safetyModeMessage":
        return "safety mode {} (code {}.{})".format(
            message["safetyModeType"], message["code"], message["argument"]
        )
    if key == "keyMessage":
        return "{}: {}".format(_text(message, "messageTitle"), _text(message))
    return _text(message)


class PrimaryMonitor(Thread):
    """
    Read-only listener on the primary port. Programs still go out over the secondary
    port, so this never writes: UR designates the last client to connect on 30001 as
    the primary one, and sending from here would fight the pendant for it.
    """

    def __init__(self, host, maxlen=50):
        Thread.__init__(self)
        self.daemon = True
        self.logger = logging.getLogger("URX Logger")
        self.host = host
        self._parser = ParserUtils()
        self._messages = deque(maxlen=maxlen)
        self._lock = Lock()
        self._trystop = False
        self._sock = None
        self._dataqueue = bytes()
        self._unreachable = False
        self.start()

    def run(self):
        while not self._trystop:
            if self._sock is None:
                self._connect()
                continue
            packet = self._read_packet()
            if packet:
                self._handle(packet)

    def _connect(self):
        try:
            self._sock = socket.create_connection((self.host, PRIMARY_PORT), timeout=2.0)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._dataqueue = bytes()
            self._unreachable = False
            self.logger.info("Listening for robot messages on %s:%s", self.host, PRIMARY_PORT)
        except OSError as ex:
            # Only the first of a run of failures is worth a line; retries are every
            # RECONNECT_DELAY for as long as the robot is up.
            if not self._unreachable:
                self.logger.warning(
                    "No robot message listener: %s:%s unreachable (%s)",
                    self.host, PRIMARY_PORT, ex,
                )
                self._unreachable = True
            self._sleep(RECONNECT_DELAY)

    def _sleep(self, seconds):
        """Sleep in slices so close() does not have to wait one out."""
        deadline = time.time() + seconds
        while time.time() < deadline and not self._trystop:
            time.sleep(0.1)

    def _drop(self, reason):
        self.logger.warning("Robot message listener reconnecting: %s", reason)
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None

    def _read_packet(self):
        """One complete packet, or None when more bytes are needed."""
        if len(self._dataqueue) >= HEADER.size:
            psize, _ = HEADER.unpack(self._dataqueue[: HEADER.size])
            if psize < HEADER.size or psize > MAX_PACKET_SIZE:
                self._drop("implausible packet size %s" % psize)
                return None
            if len(self._dataqueue) >= psize:
                packet = self._dataqueue[:psize]
                self._dataqueue = self._dataqueue[psize:]
                return packet

        try:
            self._sock.settimeout(0.5)
            chunk = self._sock.recv(4096)
        except socket.timeout:
            return None
        except OSError as ex:
            self._drop(ex)
            return None

        if not chunk:
            self._drop("peer closed connection")
            return None
        self._dataqueue += chunk
        return None

    def _handle(self, packet):
        try:
            parsed = self._parser.parse(packet)
        except ParsingException as ex:
            # Silence here would recreate the problem this listener exists to fix, so
            # report the payload even when the layout does not match.
            self.logger.info(
                "Could not parse primary packet (%s): %s", ex, printable_text(packet)
            )
            return

        for key in MESSAGE_KEYS:
            if key not in parsed:
                continue
            text = _describe(key, parsed[key])
            self.logger.info("Robot message - %s", text)
            with self._lock:
                self._messages.append((time.time(), text))

    def get_messages(self, since=None):
        """Retained messages, newest last. ``since`` is a ``time.time()`` stamp."""
        with self._lock:
            return [text for stamp, text in self._messages if since is None or stamp >= since]

    def close(self):
        self._trystop = True
        self.join(timeout=2.0)
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
