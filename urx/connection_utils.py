"""
Connection utilities for robust socket management with automatic reconnection,
metrics tracking, and cross-platform compatibility.

This module provides socket wrappers that handle:
- Automatic reconnection with exponential backoff
- Short timeouts with missed-cycle detection
- Platform-specific socket options (TCP_NODELAY, SO_KEEPALIVE)
- Sleep/roam detection via monotonic time gaps
- Comprehensive connection health metrics
"""

import socket
import time
import logging
import errno
from threading import Lock, Condition
from dataclasses import dataclass, field
from typing import Dict, Optional
from collections import deque

__author__ = "Walker"
__license__ = "LGPLv3"


@dataclass
class SocketConfig:
    """Configuration for socket connection behavior."""
    connect_timeout: float = 1.0
    recv_timeout: float = 0.3
    send_timeout: float = 1.0
    tcp_nodelay: bool = True
    so_keepalive: bool = True
    missed_cycle_threshold: int = 3  # Consecutive timeouts before reconnect


@dataclass
class ConnectionStats:
    """Metrics tracking for connection health."""
    total_connects: int = 0
    total_disconnects: int = 0
    timeout_count: int = 0
    missed_cycles: int = 0
    last_recv_time: float = 0.0
    avg_recv_duration: float = 0.0
    disconnect_reasons: Dict[str, int] = field(default_factory=dict)
    
    # Rolling average tracking
    _recv_durations: deque = field(default_factory=lambda: deque(maxlen=100))
    _lock: Lock = field(default_factory=Lock)
    
    def update_recv(self, duration: Optional[float] = None):
        """Update receive statistics."""
        with self._lock:
            self.last_recv_time = time.time()
            if duration is not None:
                self._recv_durations.append(duration)
                if self._recv_durations:
                    self.avg_recv_duration = sum(self._recv_durations) / len(self._recv_durations)
    
    def record_timeout(self):
        """Record a timeout event."""
        with self._lock:
            self.timeout_count += 1
    
    def record_disconnect(self, reason: str):
        """Record a disconnect event with reason."""
        with self._lock:
            self.total_disconnects += 1
            self.disconnect_reasons[reason] = self.disconnect_reasons.get(reason, 0) + 1
    
    def record_connect(self):
        """Record a successful connect."""
        with self._lock:
            self.total_connects += 1
    
    def get_stats_dict(self) -> dict:
        """Return stats as dictionary."""
        with self._lock:
            return {
                'total_connects': self.total_connects,
                'total_disconnects': self.total_disconnects,
                'timeout_count': self.timeout_count,
                'missed_cycles': self.missed_cycles,
                'last_recv_time': self.last_recv_time,
                'avg_recv_duration_ms': self.avg_recv_duration * 1000,
                'disconnect_reasons': dict(self.disconnect_reasons),
            }
    
    def reset(self):
        """Reset all statistics."""
        with self._lock:
            self.total_connects = 0
            self.total_disconnects = 0
            self.timeout_count = 0
            self.missed_cycles = 0
            self.last_recv_time = 0.0
            self.avg_recv_duration = 0.0
            self.disconnect_reasons.clear()
            self._recv_durations.clear()


class ReconnectBackoff:
    """Exponential backoff manager for reconnection attempts."""
    
    def __init__(self, initial: float = 0.2, multiplier: float = 2.5, cap: float = 5.0):
        self.initial = initial
        self.multiplier = multiplier
        self.cap = cap
        self.current = initial
        self._lock = Lock()
    
    def get_delay(self) -> float:
        """Get current backoff delay and increment for next time."""
        with self._lock:
            delay = self.current
            self.current = min(self.current * self.multiplier, self.cap)
            return delay
    
    def reset(self):
        """Reset backoff to initial value."""
        with self._lock:
            self.current = self.initial


def apply_socket_options(sock: socket.socket, config: SocketConfig, logger: logging.Logger, timeout: Optional[float] = None):
    """Apply socket options with graceful degradation for cross-platform compatibility.
    
    Args:
        sock: Socket to configure
        config: Socket configuration
        logger: Logger instance
        timeout: Override timeout (None to use config.recv_timeout)
    """
    try:
        # Always supported options
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1 if config.tcp_nodelay else 0)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1 if config.so_keepalive else 0)
        
        # Set timeout (use override or default to recv_timeout)
        sock.settimeout(timeout if timeout is not None else config.recv_timeout)
        
        # Platform-specific keepalive tuning (best effort)
        if config.so_keepalive:
            try:
                if hasattr(socket, 'TCP_KEEPIDLE'):  # Linux/WSL
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)
                elif hasattr(socket, 'TCP_KEEPALIVE'):  # macOS
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPALIVE, 60)
            except (OSError, AttributeError) as ex:
                logger.debug("Could not set keepalive tuning: %s", ex)
                
    except Exception as ex:
        logger.warning("Error applying socket options: %s", ex)


class RobustSocket:
    """
    Base class for robust socket connections with automatic reconnection.
    
    Features:
    - Automatic reconnection with exponential backoff
    - Sleep/roam detection via monotonic time gaps
    - Comprehensive error handling and metrics
    - Platform-specific socket options
    """
    
    def __init__(self, host: str, port: int, config: SocketConfig, name: str = "RobustSocket"):
        self.host = host
        self.port = port
        self.config = config
        self.name = name
        self.logger = logging.getLogger('URX Logger')
        
        self._socket: Optional[socket.socket] = None
        self._connected = False
        self._lock = Lock()
        self._backoff = ReconnectBackoff()
        self._stats = ConnectionStats()
        
        self._last_monotonic = time.monotonic()
        
    def connect(self):
        """Establish connection to robot."""
        with self._lock:
            self._close_socket()
            
            try:
                self.logger.debug("%s: Connecting to %s:%d", self.name, self.host, self.port)
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                # Apply socket options with connect timeout before connect
                apply_socket_options(self._socket, self.config, self.logger, timeout=self.config.connect_timeout)
                
                # Connect
                self._socket.connect((self.host, self.port))
                
                # After successful connect, set to recv_timeout for normal operations
                self._socket.settimeout(self.config.recv_timeout)
                
                self._connected = True
                self._backoff.reset()
                self._stats.record_connect()
                self._last_monotonic = time.monotonic()
                
                self.logger.debug("%s: Connected successfully", self.name)
                
            except Exception as ex:
                self.logger.error("%s: Connection failed: %s", self.name, ex)
                self._stats.record_disconnect(f"connect_failed_{type(ex).__name__}")
                self._close_socket()
                raise
    
    def _close_socket(self):
        """Close socket if open."""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        self._connected = False
    
    def close(self):
        """Close connection."""
        with self._lock:
            self._close_socket()
    
    def is_connected(self) -> bool:
        """Check if socket is connected."""
        return self._connected and self._socket is not None
    
    def detect_sleep_resume(self) -> bool:
        """Return True if >5s monotonic gap detected (system sleep/resume)."""
        now = time.monotonic()
        gap = now - self._last_monotonic
        self._last_monotonic = now
        
        if gap > 5.0:
            self.logger.warning("%s: Detected %0.1fs time gap (system sleep?)", self.name, gap)
            self._stats.record_disconnect("sleep_detected")
            return True
        return False
    
    def _reconnect(self):
        """Perform reconnection with backoff."""
        delay = self._backoff.get_delay()
        self.logger.info("%s: Reconnecting in %.2fs...", self.name, delay)
        time.sleep(delay)
        self.connect()
    
    def get_stats(self) -> dict:
        """Get connection statistics."""
        return self._stats.get_stats_dict()
    
    def reset_stats(self):
        """Reset connection statistics."""
        self._stats.reset()


class StateReaderSocket(RobustSocket):
    """
    Read-only socket for continuous state streaming from robot.
    
    Features:
    - Short recv timeout (0.3-0.5s)
    - Missed-cycle detection with automatic reconnection
    - Drop malformed packets and continue
    - Never blocks on write operations
    """
    
    def __init__(self, host: str, port: int, config: SocketConfig):
        super().__init__(host, port, config, name="StateReader")
        self._consecutive_timeouts = 0
        self._recv_lock = Lock()
    
    def recv_data(self, size: int) -> Optional[bytes]:
        """
        Receive data with timeout and reconnection logic.
        
        Returns:
            bytes: Received data
            None: On timeout (caller should retry)
            
        Raises:
            Exception: On unrecoverable errors
        """
        # Check for sleep/resume
        if self.detect_sleep_resume():
            self.logger.info("%s: Sleep detected, reconnecting", self.name)
            self._reconnect()
        
        # Ensure connected
        if not self.is_connected():
            self.logger.warning("%s: Not connected, attempting connection", self.name)
            try:
                self.connect()
            except Exception as ex:
                self.logger.error("%s: Connect failed: %s", self.name, ex)
                return None
        
        try:
            start_time = time.time()
            with self._recv_lock:
                if self._socket is None:
                    self.logger.warning("%s: Socket is None during recv", self.name)
                    return None
                data = self._socket.recv(size)
            recv_duration = time.time() - start_time
            
            if not data:
                # Peer closed connection
                self.logger.warning("%s: Peer closed connection", self.name)
                self._stats.record_disconnect("peer_closed")
                self._connected = False
                self._reconnect()
                return None
            
            # Success
            self._consecutive_timeouts = 0
            self._stats.update_recv(recv_duration)
            return data
            
        except socket.timeout:
            self._stats.record_timeout()
            self._consecutive_timeouts += 1
            
            if self._consecutive_timeouts >= self.config.missed_cycle_threshold:
                self.logger.warning(
                    "%s: %d consecutive timeouts, reconnecting",
                    self.name, self._consecutive_timeouts
                )
                self._stats.record_disconnect("missed_cycles")
                self._stats.missed_cycles += 1
                self._consecutive_timeouts = 0
                self._connected = False
                try:
                    self._reconnect()
                except Exception as ex:
                    self.logger.error("%s: Reconnect failed: %s", self.name, ex)
            
            return None
            
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as ex:
            self.logger.warning("%s: Connection reset: %s", self.name, ex)
            self._stats.record_disconnect("connection_reset")
            self._connected = False
            self._consecutive_timeouts = 0
            try:
                self._reconnect()
            except Exception as reconnect_ex:
                self.logger.error("%s: Reconnect failed: %s", self.name, reconnect_ex)
            return None
            
        except OSError as ex:
            if ex.errno in (errno.ECONNRESET, errno.ECONNABORTED, errno.EPIPE):
                self.logger.warning("%s: OS connection error: %s", self.name, ex)
                self._stats.record_disconnect(f"os_error_{ex.errno}")
                self._connected = False
                self._consecutive_timeouts = 0
                try:
                    self._reconnect()
                except Exception as reconnect_ex:
                    self.logger.error("%s: Reconnect failed: %s", self.name, reconnect_ex)
                return None
            else:
                raise


class CommandSocket(RobustSocket):
    """
    Write-only socket for sending URScript commands to robot.
    
    Features:
    - Queue-based sending (non-blocking to caller)
    - Handle partial sends
    - Automatic command termination with newline
    - Reconnect and retry on connection loss
    """
    
    def __init__(self, host: str, port: int, config: SocketConfig):
        super().__init__(host, port, config, name="CommandSocket")
        self._send_lock = Lock()
        self._pending_command = None
    
    def send_command(self, command, retry: bool = True) -> bool:
        """
        Send command to robot.
        
        Args:
            command: URScript command string (str or bytes)
            retry: Whether to retry once on failure
            
        Returns:
            True if sent successfully, False otherwise
        """
        # Store original command for potential retry
        original_command = command
        
        # Check for sleep/resume
        if self.detect_sleep_resume():
            self.logger.info("%s: Sleep detected, reconnecting", self.name)
            try:
                self._reconnect()
            except Exception as ex:
                self.logger.error("%s: Reconnect after sleep failed: %s", self.name, ex)
                return False
        
        # Ensure connected
        if not self.is_connected():
            self.logger.warning("%s: Not connected, attempting connection", self.name)
            try:
                self.connect()
            except Exception as ex:
                self.logger.error("%s: Connect failed: %s", self.name, ex)
                return False
        
        # Ensure command ends with newline
        if not isinstance(command, bytes):
            command = command.encode()
        if not command.endswith(b'\n'):
            command = command + b'\n'
        
        try:
            with self._send_lock:
                if self._socket is None:
                    self.logger.warning("%s: Socket is None during send", self.name)
                    return False
                # Send all data (handle partial sends)
                total_sent = 0
                while total_sent < len(command):
                    sent = self._socket.send(command[total_sent:])
                    if sent == 0:
                        raise RuntimeError("Socket connection broken")
                    total_sent += sent
                
                self.logger.debug("%s: Sent %d bytes", self.name, total_sent)
                return True
                
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as ex:
            self.logger.warning("%s: Connection error during send: %s", self.name, ex)
            self._stats.record_disconnect("send_connection_reset")
            self._connected = False
            
            if retry:
                self.logger.info("%s: Retrying send after reconnect", self.name)
                try:
                    self._reconnect()
                    return self.send_command(original_command, retry=False)
                except Exception as reconnect_ex:
                    self.logger.error("%s: Reconnect failed: %s", self.name, reconnect_ex)
                    return False
            return False
            
        except OSError as ex:
            if ex.errno in (errno.ECONNRESET, errno.ECONNABORTED, errno.EPIPE):
                self.logger.warning("%s: OS error during send: %s", self.name, ex)
                self._stats.record_disconnect(f"send_os_error_{ex.errno}")
                self._connected = False
                
                if retry:
                    self.logger.info("%s: Retrying send after reconnect", self.name)
                    try:
                        self._reconnect()
                        return self.send_command(original_command, retry=False)
                    except Exception as reconnect_ex:
                        self.logger.error("%s: Reconnect failed: %s", self.name, reconnect_ex)
                        return False
                return False
            else:
                raise
        
        except Exception as ex:
            self.logger.error("%s: Unexpected error during send: %s", self.name, ex)
            return False


class IKQuerySocket(RobustSocket):
    """
    Ephemeral socket for isolated IK queries.
    
    Creates temporary connections for IK computation requests,
    preventing interference with state reading and command execution.
    """
    
    def __init__(self, host: str, port: int, config: SocketConfig):
        super().__init__(host, port, config, name="IKQuerySocket")
    
    def send_query(self, urscript_program: str) -> bool:
        """
        Send IK query URScript program.
        
        Args:
            urscript_program: Complete URScript program for IK query
            
        Returns:
            True if sent successfully, False otherwise
        """
        # Ensure connected
        if not self.is_connected():
            try:
                self.connect()
            except Exception as ex:
                self.logger.error("%s: Connect failed: %s", self.name, ex)
                return False
        
        # Ensure program ends with newline
        if not isinstance(urscript_program, bytes):
            urscript_program = urscript_program.encode()
        if not urscript_program.endswith(b'\n'):
            urscript_program = urscript_program + b'\n'
        
        try:
            if self._socket is None:
                self.logger.warning("%s: Socket is None during send", self.name)
                return False
            # Send all data
            total_sent = 0
            while total_sent < len(urscript_program):
                sent = self._socket.send(urscript_program[total_sent:])
                if sent == 0:
                    raise RuntimeError("Socket connection broken")
                total_sent += sent
            
            self.logger.debug("%s: Sent IK query (%d bytes)", self.name, total_sent)
            return True
            
        except Exception as ex:
            self.logger.error("%s: Error sending IK query: %s", self.name, ex)
            self._stats.record_disconnect(f"ik_send_error_{type(ex).__name__}")
            self._connected = False
            return False
