"""
This file contains 2 classes:
    - ParseUtils containing utilies to parse data from UR robot
    - SecondaryMonitor, a class opening a socket to the robot and with methods to
            access data and send programs to the robot
Both use data from the secondary port of the URRobot.
Only the last connected socket on 3001 is the primary client !!!!
So do not rely on it unless you know no other client is running (Hint the UR java interface is a client...)
http://support.universal-robots.com/Technical/PrimaryAndSecondaryClientInterface
"""

from threading import Thread, Condition, Lock
import logging
import os
import struct
import socket
from copy import copy
import time

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2013, Sintef Raufoss Manufacturing"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "LGPLv3"


class ParsingException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, *args)


class Program(object):
    def __init__(self, prog):
        self.program = prog
        self.condition = Condition()

    def __str__(self):
        return "Program({})".format(self.program)

    __repr__ = __str__


class TimeoutException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, *args)


class ParserUtils(object):
    def __init__(self):
        self.logger = logging.getLogger('URX Logger')
        self.version = (0, 0)

    def parse(self, data):
        """
        parse a packet from the UR socket and return a dictionary with the data
        """
        allData = {}
        while data:
            psize, ptype, pdata, data = self.analyze_header(data)
            if ptype == 16:
                allData["SecondaryClientData"] = self._get_data(
                    pdata, "!iB", ("size", "type")
                )
                data = (pdata + data)[5:]
            elif ptype == 0:
                if psize == 38:
                    self.version = (3, 0)
                    allData["RobotModeData"] = self._get_data(
                        pdata,
                        "!IBQ???????BBdd",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "isRobotConnected",
                            "isRealRobotEnabled",
                            "isPowerOnRobot",
                            "isEmergencyStopped",
                            "isSecurityStopped",
                            "isProgramRunning",
                            "isProgramPaused",
                            "robotMode",
                            "controlMode",
                            "speedFraction",
                            "speedScaling",
                        ),
                    )
                elif psize == 46:
                    self.version = (3, 2)
                    allData["RobotModeData"] = self._get_data(
                        pdata,
                        "!IBQ???????BBdd",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "isRobotConnected",
                            "isRealRobotEnabled",
                            "isPowerOnRobot",
                            "isEmergencyStopped",
                            "isSecurityStopped",
                            "isProgramRunning",
                            "isProgramPaused",
                            "robotMode",
                            "controlMode",
                            "speedFraction",
                            "speedScaling",
                            "speedFractionLimit",
                        ),
                    )
                elif psize == 47:
                    self.version = (3, 5)
                    allData["RobotModeData"] = self._get_data(
                        pdata,
                        "!IBQ???????BBddc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "isRobotConnected",
                            "isRealRobotEnabled",
                            "isPowerOnRobot",
                            "isEmergencyStopped",
                            "isSecurityStopped",
                            "isProgramRunning",
                            "isProgramPaused",
                            "robotMode",
                            "controlMode",
                            "speedFraction",
                            "speedScaling",
                            "speedFractionLimit",
                            "reservedByUR",
                        ),
                    )
                else:
                    allData["RobotModeData"] = self._get_data(
                        pdata,
                        "!iBQ???????Bd",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "isRobotConnected",
                            "isRealRobotEnabled",
                            "isPowerOnRobot",
                            "isEmergencyStopped",
                            "isSecurityStopped",
                            "isProgramRunning",
                            "isProgramPaused",
                            "robotMode",
                            "speedFraction",
                        ),
                    )
            elif ptype == 1:
                tmpstr = ["size", "type"]
                for i in range(0, 6):
                    tmpstr += [
                        "q_actual%s" % i,
                        "q_target%s" % i,
                        "qd_actual%s" % i,
                        "I_actual%s" % i,
                        "V_actual%s" % i,
                        "T_motor%s" % i,
                        "T_micro%s" % i,
                        "jointMode%s" % i,
                    ]

                allData["JointData"] = self._get_data(
                    pdata,
                    "!iB dddffffB dddffffB dddffffB dddffffB dddffffB dddffffB",
                    tmpstr,
                )

            elif ptype == 4:
                if self.version < (3, 2):
                    allData["CartesianInfo"] = self._get_data(
                        pdata,
                        "iBdddddd",
                        ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz"),
                    )
                else:
                    allData["CartesianInfo"] = self._get_data(
                        pdata,
                        "iBdddddddddddd",
                        (
                            "size",
                            "type",
                            "X",
                            "Y",
                            "Z",
                            "Rx",
                            "Ry",
                            "Rz",
                            "tcpOffsetX",
                            "tcpOffsetY",
                            "tcpOffsetZ",
                            "tcpOffsetRx",
                            "tcpOffsetRy",
                            "tcpOffsetRz",
                        ),
                    )
            elif ptype == 5:
                allData["LaserPointer(OBSOLETE)"] = self._get_data(
                    pdata, "iBddd", ("size", "type")
                )
            elif ptype == 3:
                if self.version >= (3, 0):
                    fmt = "iBiibbddbbddffffBBb"
                else:
                    fmt = "iBhhbbddbbddffffBBb"

                allData["MasterBoardData"] = self._get_data(
                    pdata,
                    fmt,
                    (
                        "size",
                        "type",
                        "digitalInputBits",
                        "digitalOutputBits",
                        "analogInputRange0",
                        "analogInputRange1",
                        "analogInput0",
                        "analogInput1",
                        "analogInputDomain0",
                        "analogInputDomain1",
                        "analogOutput0",
                        "analogOutput1",
                        "masterBoardTemperature",
                        "robotVoltage48V",
                        "robotCurrent",
                        "masterIOCurrent",
                    ),
                )
            elif ptype == 2:
                allData["ToolData"] = self._get_data(
                    pdata,
                    "iBbbddfBffB",
                    (
                        "size",
                        "type",
                        "analoginputRange2",
                        "analoginputRange3",
                        "analogInput2",
                        "analogInput3",
                        "toolVoltage48V",
                        "toolOutputVoltage",
                        "toolCurrent",
                        "toolTemperature",
                        "toolMode",
                    ),
                )
            elif ptype == 9:
                continue
            elif ptype == 8 and self.version >= (3, 2):
                allData["AdditionalInfo"] = self._get_data(
                    pdata,
                    "iB??",
                    ("size", "type", "teachButtonPressed", "teachButtonEnabled"),
                )
            elif ptype == 7 and self.version >= (3, 2):
                allData["ForceModeData"] = self._get_data(
                    pdata,
                    "iBddddddd",
                    ("size", "type", "x", "y", "z", "rx", "ry", "rz", "robotDexterity"),
                )
            elif ptype == 20:
                tmp = self._get_data(
                    pdata,
                    "!iB Qbb",
                    ("size", "type", "timestamp", "source", "robotMessageType"),
                )
                if tmp["robotMessageType"] == 3:
                    allData["VersionMessage"] = self._get_data(
                        pdata,
                        "!iBQbb bAbBBiAb",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "projectNameSize",
                            "projectName",
                            "majorVersion",
                            "minorVersion",
                            "svnRevision",
                            "buildDate",
                        ),
                    )
                elif tmp["robotMessageType"] == 6:
                    allData["robotCommMessage"] = self._get_data(
                        pdata,
                        "!iBQbb iiAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "code",
                            "argument",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 1:
                    allData["labelMessage"] = self._get_data(
                        pdata,
                        "!iBQbb iAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "id",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 2:
                    allData["popupMessage"] = self._get_data(
                        pdata,
                        "!iBQbb ??BAcAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "warning",
                            "error",
                            "titleSize",
                            "messageTitle",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 0:
                    allData["messageText"] = self._get_data(
                        pdata,
                        "!iBQbb Ac",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 8:
                    allData["varMessage"] = self._get_data(
                        pdata,
                        "!iBQbb iiBAcAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "code",
                            "argument",
                            "titleSize",
                            "messageTitle",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 7:
                    allData["keyMessage"] = self._get_data(
                        pdata,
                        "!iBQbb iiBAcAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "code",
                            "argument",
                            "titleSize",
                            "messageTitle",
                            "messageText",
                        ),
                    )
                elif tmp["robotMessageType"] == 5:
                    allData["keyMessage"] = self._get_data(
                        pdata,
                        "!iBQbb iiAc",
                        (
                            "size",
                            "type",
                            "timestamp",
                            "source",
                            "robotMessageType",
                            "code",
                            "argument",
                            "messageText",
                        ),
                    )
                else:
                    self.logger.debug("Message type parser not implemented %s", tmp)
            else:
                self.logger.debug("Unknown packet type %s with size %s", ptype, psize)

        return allData

    def _get_data(self, data, fmt, names):
        """
        fill data into a dictionary
            data is data from robot packet
            fmt is struct format, but with added A for arrays and no support for numerical in fmt
            names args are strings used to store values
        """
        tmpdata = copy(data)
        fmt = fmt.strip()
        d = dict()
        i = 0
        j = 0
        while j < len(fmt) and i < len(names):
            f = fmt[j]
            if f in (" ", "!", ">", "<"):
                j += 1
            elif f == "A":
                if j == len(fmt) - 2:
                    arraysize = len(tmpdata)
                else:
                    asn = names[i - 1]
                    if not asn.endswith("Size"):
                        raise ParsingException(
                            "Error, array without size ! %s %s" % (asn, i)
                        )
                    else:
                        arraysize = d[asn]
                d[names[i]] = tmpdata[0:arraysize]
                tmpdata = tmpdata[arraysize:]
                j += 2
                i += 1
            else:
                fmtsize = struct.calcsize(fmt[j])
                if len(tmpdata) < fmtsize:
                    raise ParsingException(
                        "Error, length of data smaller than advertized: ",
                        len(tmpdata),
                        fmtsize,
                        "for names ",
                        names,
                        f,
                        i,
                        j,
                    )
                d[names[i]] = struct.unpack("!" + f, tmpdata[0:fmtsize])[0]
                tmpdata = tmpdata[fmtsize:]
                j += 1
                i += 1
        return d

    def get_header(self, data):
        return struct.unpack("!iB", data[0:5])

    def analyze_header(self, data):
        """
        read first 5 bytes and return complete packet
        """
        if len(data) < 5:
            raise ParsingException(
                "Packet size %s smaller than header size (5 bytes)" % len(data)
            )
        else:
            psize, ptype = self.get_header(data)
            if psize < 5:
                raise ParsingException(
                    "Error, declared length of data smaller than its own header(5): ",
                    psize,
                )
            elif psize > len(data):
                raise ParsingException(
                    "Error, length of data smaller (%s) than declared (%s)"
                    % (len(data), psize)
                )
        return psize, ptype, data[:psize], data[psize:]

    def find_first_packet(self, data):
        """
        find the first complete packet in a string
        returns None if none found
        """
        counter = 0
        limit = 10
        while True:
            if len(data) >= 5:
                psize, ptype = self.get_header(data)
                if psize < 5 or psize > 2000 or ptype != 16:
                    data = data[1:]
                    counter += 1
                    if counter > limit:
                        self.logger.warning(
                            "tried %s times to find a packet in data, advertised packet size: %s, type: %s",
                            counter,
                            psize,
                            ptype,
                        )
                        self.logger.warning("Data length: %s", len(data))
                        limit = limit * 10
                elif len(data) >= psize:
                    self.logger.debug(
                        "Got packet with size %s and type %s", psize, ptype
                    )
                    if counter:
                        self.logger.info(
                            "Remove %s bytes of garbage at begining of packet", counter
                        )
                    return (data[:psize], data[psize:])
                else:
                    self.logger.debug(
                        "Packet is not complete, advertised size is %s, received size is %s, type is %s",
                        psize,
                        len(data),
                        ptype,
                    )
                    return None
            else:
                return None


class SecondaryMonitor(Thread):
    """
    Monitor data from secondary port and send programs to robot
    """

    def __init__(self, host):
        Thread.__init__(self)
        self.logger = logging.getLogger('URX Logger')
        self._parser = ParserUtils()
        self._dict = {}
        self._dictLock = Lock()
        self.host = host
        self.secondary_port = 30002
        
        # Simple socket creation
        self._s_secondary = socket.create_connection(
            (self.host, self.secondary_port), timeout=2.0
        )
        self._s_secondary.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        self._prog_queue = []
        self._prog_queue_lock = Lock()
        self._dataqueue = bytes()
        self._trystop = False
        self.running = False
        self._dataEvent = Condition()
        self.lastpacket_timestamp = 0
        self._consecutive_timeouts = 0

        self.start()
        try:
            self.wait()
        except TimeoutException as ex:
            self.close()
            raise ex

    def send_program(self, prog):
        """
        send program to robot in URRobot format
        If another program is send while a program is running the first program is aborded.
        """
        prog = prog.strip()
        self.logger.debug("Enqueueing program: %s", prog)
        if not isinstance(prog, bytes):
            prog = prog.encode()

        data = Program(prog + b"\n")
        with data.condition:
            with self._prog_queue_lock:
                self._prog_queue.append(data)
            data.condition.wait()
            self.logger.debug("program sent: %s", data)

    def run(self):
        """
        check program execution status in the secondary client data packet we get from the robot
        This interface uses only data from the secondary client interface (see UR doc)
        Only the last connected client is the primary client,
        so this is not guaranted and we cannot rely on information to the primary client.
        """
        while not self._trystop:
            # Send queued programs
            with self._prog_queue_lock:
                if len(self._prog_queue) > 0:
                    data = self._prog_queue.pop(0)
                    try:
                        self._s_secondary.send(data.program)
                    except (ConnectionResetError, BrokenPipeError, OSError) as ex:
                        # Connection lost during send, will reconnect in _get_data
                        self.logger.warning("Error sending program: %s", ex)
                    with data.condition:
                        data.condition.notify_all()

            # Read and parse data
            data = self._get_data()
            if not data:
                continue
                
            try:
                tmpdict = self._parser.parse(data)
                with self._dictLock:
                    self._dict = tmpdict
            except ParsingException as ex:
                self.logger.warning("Error parsing one packet from urrobot: %s", ex)
                continue

            if "RobotModeData" not in self._dict:
                self.logger.warning(
                    "Got a packet from robot without RobotModeData, strange ..."
                )
                continue

            self.lastpacket_timestamp = time.time()

            # Check robot running state
            rmode = 7 if self._parser.version >= (3, 0) else 0
            
            if (self._dict["RobotModeData"]["robotMode"] == rmode and
                self._dict["RobotModeData"]["isRealRobotEnabled"] is True and
                self._dict["RobotModeData"]["isEmergencyStopped"] is False and
                self._dict["RobotModeData"]["isSecurityStopped"] is False and
                self._dict["RobotModeData"]["isRobotConnected"] is True and
                self._dict["RobotModeData"]["isPowerOnRobot"] is True):
                self.running = True
            else:
                if self.running:
                    self.logger.error(
                        "Robot not running: " + str(self._dict["RobotModeData"])
                    )
                self.running = False
                
            with self._dataEvent:
                self._dataEvent.notifyAll()

    def _attempt_reconnect(self):
        """
        Attempt to reconnect with exponential backoff
        """
        attempt = 0
        max_attempts = 10
        
        while attempt < max_attempts and not self._trystop:
            attempt += 1
            try:
                # Close old socket
                try:
                    self._s_secondary.close()
                except:
                    pass
                
                # Wait progressively longer: 0.5s, 1s, 2s, 3s, max 5s
                wait_time = min(0.5 * attempt, 5.0)
                self.logger.info(
                    "Reconnect attempt %d/%d in %.1fs...", 
                    attempt, max_attempts, wait_time
                )
                time.sleep(wait_time)
                
                # Try to reconnect
                self._s_secondary = socket.create_connection(
                    (self.host, self.secondary_port), timeout=2.0
                )
                self._s_secondary.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                )
                
                self.logger.info("Reconnected successfully")
                self._dataqueue = bytes()  # Clear buffer
                return  # Success
                
            except Exception as reconnect_ex:
                self.logger.debug(
                    "Reconnect attempt %d failed: %s", 
                    attempt, reconnect_ex
                )
                if attempt >= max_attempts:
                    self.logger.error(
                        "Failed to reconnect after %d attempts", 
                        max_attempts
                    )
                    raise

    def _get_data(self):
        """
        Returns a complete packet, handles reconnection on errors
        """
        while not self._trystop:
            ans = self._parser.find_first_packet(self._dataqueue[:])
            
            if ans:
                self._dataqueue = ans[1]
                self._consecutive_timeouts = 0  # Reset on successful packet
                self.logger.debug("Found packet of size {}".format(len(ans[0])))
                return ans[0]
            
            # Need more data
            try:
                self._s_secondary.settimeout(0.5)  # Short timeout for recv
                tmp = self._s_secondary.recv(1024)
                if not tmp:
                    # Socket closed by peer
                    raise ConnectionResetError("Peer closed connection")
                self._dataqueue += tmp
                self._consecutive_timeouts = 0  # Reset on successful recv
                
            except socket.timeout:
                # Increment timeout counter
                self._consecutive_timeouts += 1
                
                # Too many consecutive timeouts = connection is dead
                if self._consecutive_timeouts >= 5:  # 20 * 0.5s = 10 seconds
                    self.logger.warning(
                        "No data received for %d timeouts (%.1fs), connection appears dead",
                        self._consecutive_timeouts,
                        self._consecutive_timeouts * 0.5
                    )
                    self._consecutive_timeouts = 0
                    # Instead of raising, call reconnection directly
                    self._attempt_reconnect()
                    continue  # After reconnect, continue loop
                
                # Normal timeout, just retry
                continue
                
            except (ConnectionResetError, ConnectionAbortedError, 
                    ConnectionRefusedError, BrokenPipeError, OSError) as ex:
                # Connection lost - try to reconnect
                self._consecutive_timeouts = 0
                self.logger.warning("Connection error: %s, reconnecting...", ex)
                self._attempt_reconnect()
                continue

    def wait(self, timeout=60):
        """
        wait for next data packet from robot
        """
        tstamp = self.lastpacket_timestamp
        with self._dataEvent:
            self._dataEvent.wait(timeout)
            if tstamp == self.lastpacket_timestamp:
                raise TimeoutException(
                    "Did not receive a valid data packet from robot in {}".format(
                        timeout
                    )
                )

    def get_cartesian_info(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            if "CartesianInfo" in self._dict:
                return self._dict["CartesianInfo"]
            else:
                return None

    def get_all_data(self, wait=False):
        """
        return last data obtained from robot in dictionnary format
        """
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict.copy()

    def get_joint_data(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            if "JointData" in self._dict:
                return self._dict["JointData"]
            else:
                return None

    def get_digital_out(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            output = self._dict["MasterBoardData"]["digitalOutputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_out_bits(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["digitalOutputBits"]

    def get_digital_in(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            output = self._dict["MasterBoardData"]["digitalInputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_in_bits(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["digitalInputBits"]

    def get_analog_in(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["analogInput" + str(nb)]

    def get_analog_inputs(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return (
                self._dict["MasterBoardData"]["analogInput0"],
                self._dict["MasterBoardData"]["analogInput1"],
            )

    def is_program_running(self, wait=False):
        """
        return True if robot is executing a program
        Rmq: The refresh rate is only 10Hz so the information may be outdated
        """
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["RobotModeData"]["isProgramRunning"]
    
    def _retry_ik_operation(self, operation_func, operation_name, 
                            max_retries=5, initial_delay=0.5):
        """
        Retry IK operations with simple backoff
        """
        last_exception = None
        
        for attempt in range(max_retries + 1):
            try:
                if attempt > 0:
                    delay = initial_delay * attempt  # Linear: 0.5s, 1s, 1.5s, 2s, 2.5s
                    self.logger.info(
                        "%s: Retry %d/%d after %.1fs", 
                        operation_name, attempt, max_retries, delay
                    )
                    time.sleep(delay)
                
                return operation_func()
                
            except socket.timeout as ex:
                last_exception = ex
                self.logger.warning(
                    "%s: Timeout on attempt %d/%d",
                    operation_name, attempt + 1, max_retries + 1
                )
                if attempt == max_retries:
                    break
            except Exception as ex:
                # Non-timeout exceptions are not retried
                self.logger.error("%s: Non-timeout error: %s", operation_name, ex)
                raise
        
        # All retries exhausted
        self.logger.error(
            "%s: All %d attempts failed",
            operation_name, max_retries + 1
        )
        raise last_exception

    def get_inverse_kin(self, pose, qnear=None, maxPositionError=1e-10, 
                        maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Calculate inverse kinematics for a given pose with automatic retry on timeout.
        Returns joint positions that achieve the specified tool pose.
        
        Retries up to 5 times with linear backoff on timeout.
        
        Parameters:
            pose: tool pose as list [x, y, z, rx, ry, rz]
            qnear: list of joint positions for preferred solution (optional)
            maxPositionError: maximum allowed position error (default 1e-10)
            maxOrientationError: maximum allowed orientation error (default 1e-10)
            tcp: tcp offset pose or 'active_tcp' string (default 'active_tcp')
        
        Returns:
            list of 6 joint positions [j0, j1, j2, j3, j4, j5]
        
        Raises:
            Exception if no IK solution found or timeout after all retries
        """
        return self._retry_ik_operation(
            lambda: self._compute_inverse_kin(pose, qnear, maxPositionError, 
                                             maxOrientationError, tcp),
            "get_inverse_kin"
        )
    
    def _callback_host(self):
        """
        The address the robot is told to dial back on for IK results.
        URX_CALLBACK_HOST wins: behind a NAT the discovered address is the local
        one, which the robot cannot route to.
        """
        override = os.environ.get("URX_CALLBACK_HOST")
        if override:
            return override

        try:
            temp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_sock.connect((self.host, self.secondary_port))
            local_ip = temp_sock.getsockname()[0]
            temp_sock.close()
        except Exception:
            return self.host

        # Warn if WSL/container detected
        if local_ip.startswith('172.'):
            self.logger.warning(
                "Detected internal IP %s - set URX_CALLBACK_HOST if IK times out",
                local_ip
            )
        return local_ip

    def _compute_inverse_kin(self, pose, qnear=None, maxPositionError=1e-10,
                            maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Core inverse kinematics computation logic.
        """
        # Create temporary server to receive result
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        FIXED_PORT = 50001
        server_socket.bind(('0.0.0.0', FIXED_PORT))
        server_socket.listen(1)
        server_socket.settimeout(5.0)  # Increased from 5.0 for WSL

        port = server_socket.getsockname()[1]

        local_ip = self._callback_host()

        self.logger.info("Listening for IK result on %s:%s", local_ip, port)
        
        client_socket = None
        try:
            # Format pose as URScript pose
            pose_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*pose)
            
            # Build URScript program
            prog_lines = []
            
            # Call get_inverse_kin with appropriate parameters
            if qnear is not None:
                qnear_str = "[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*qnear)
                if tcp == 'active_tcp':
                    ik_call = "get_inverse_kin({}, {}, {:.10f}, {:.10f})".format(
                        pose_str, qnear_str, maxPositionError, maxOrientationError
                    )
                else:
                    tcp_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*tcp)
                    ik_call = "get_inverse_kin({}, {}, {:.10f}, {:.10f}, {})".format(
                        pose_str, qnear_str, maxPositionError, maxOrientationError, tcp_str
                    )
            else:
                if tcp == 'active_tcp':
                    ik_call = "get_inverse_kin({}, maxPositionError={:.10f}, maxOrientationError={:.10f})".format(
                        pose_str, maxPositionError, maxOrientationError
                    )
                else:
                    tcp_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*tcp)
                    ik_call = "get_inverse_kin({}, maxPositionError={:.10f}, maxOrientationError={:.10f}, tcp={})".format(
                        pose_str, maxPositionError, maxOrientationError, tcp_str
                    )
            
            prog_lines.append("def get_ik_program():")
            prog_lines.append("  joint_result = {}".format(ik_call))
            prog_lines.append("  socket_open(\"{}\", {})".format(local_ip, port))
            prog_lines.append("  socket_send_line(joint_result[0])")
            prog_lines.append("  socket_send_line(joint_result[1])")
            prog_lines.append("  socket_send_line(joint_result[2])")
            prog_lines.append("  socket_send_line(joint_result[3])")
            prog_lines.append("  socket_send_line(joint_result[4])")
            prog_lines.append("  socket_send_line(joint_result[5])")
            prog_lines.append("  socket_close()")
            prog_lines.append("end")
            prog_lines.append("get_ik_program()")
            
            prog = "\n".join(prog_lines)
            self.logger.info("Sending IK program: %s", prog)
            
            # Send the program
            self.send_program(prog)
            
            # Wait for connection from robot
            self.logger.info("Waiting for robot to connect...")
            client_socket, addr = server_socket.accept()
            self.logger.info("Robot connected from %s", addr)
            
            # Receive the joint values
            joint_values = []
            client_socket.settimeout(5.0)  # Increased from 2.0 for WSL
            data = b""
            
            for i in range(6):
                # Read until we get a newline
                while b"\n" not in data:
                    chunk = client_socket.recv(1024)
                    if not chunk:
                        raise Exception("Connection closed before receiving all joint values")
                    data += chunk
                
                # Extract one line
                line, data = data.split(b"\n", 1)
                joint_value = float(line.decode().strip())
                joint_values.append(joint_value)
                self.logger.info("Received joint %s: %s", i, joint_value)
            
            client_socket.close()
            self.logger.info("IK result: %s", joint_values)
            return joint_values
            
        except socket.timeout:
            self.logger.info("Timeout waiting for IK result (WSL?)")
            raise Exception("Timeout waiting for inverse kinematics result")
        except Exception as ex:
            self.logger.info("Error getting inverse kinematics: %s", ex)
            raise
        finally:
            if client_socket:
                try:
                    client_socket.close()
                except:
                    pass
            server_socket.close()

    def get_inverse_kin_has_solution(self, pose, qnear=None, maxPositionError=1e-10,
                                       maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Check if get_inverse_kin has a solution for a given pose with automatic retry on timeout.
        Returns boolean (True) or (False).
        
        Retries up to 5 times with linear backoff on timeout.
        
        Parameters:
            pose: tool pose as list [x, y, z, rx, ry, rz]
            qnear: list of joint positions for preferred solution (optional)
            maxPositionError: maximum allowed position error (default 1e-10)
            maxOrientationError: maximum allowed orientation error (default 1e-10)
            tcp: tcp offset pose or 'active_tcp' string (default 'active_tcp')
        
        Returns:
            bool: True if IK solution exists, False otherwise
        
        Raises:
            Exception if timeout or communication error after all retries
        """
        return self._retry_ik_operation(
            lambda: self._compute_inverse_kin_has_solution(pose, qnear, maxPositionError,
                                                           maxOrientationError, tcp),
            "get_inverse_kin_has_solution"
        )
    
    def _compute_inverse_kin_has_solution(self, pose, qnear=None, maxPositionError=1e-10,
                                          maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Core inverse kinematics solution check logic.
        """
        # Create temporary server to receive result
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Fixed, not ephemeral: an ephemeral port cannot be published from a container.
        FIXED_PORT = 50002
        server_socket.bind(('0.0.0.0', FIXED_PORT))
        server_socket.listen(1)
        server_socket.settimeout(10.0)  # Increased from 5.0 for WSL

        port = server_socket.getsockname()[1]

        local_ip = self._callback_host()

        self.logger.info("Listening for IK solution check on %s:%s", local_ip, port)
        
        client_socket = None
        try:
            # Format pose as URScript pose
            pose_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*pose)
            
            # Build URScript program
            prog_lines = []
            
            # Call get_inverse_kin_has_solution with appropriate parameters
            if qnear is not None:
                qnear_str = "[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*qnear)
                if tcp == 'active_tcp':
                    ik_call = "get_inverse_kin_has_solution({}, {}, {:.10f}, {:.10f})".format(
                        pose_str, qnear_str, maxPositionError, maxOrientationError
                    )
                else:
                    tcp_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*tcp)
                    ik_call = "get_inverse_kin_has_solution({}, {}, {:.10f}, {:.10f}, {})".format(
                        pose_str, qnear_str, maxPositionError, maxOrientationError, tcp_str
                    )
            else:
                if tcp == 'active_tcp':
                    ik_call = "get_inverse_kin_has_solution({}, maxPositionError={:.10f}, maxOrientationError={:.10f})".format(
                        pose_str, maxPositionError, maxOrientationError
                    )
                else:
                    tcp_str = "p[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(*tcp)
                    ik_call = "get_inverse_kin_has_solution({}, maxPositionError={:.10f}, maxOrientationError={:.10f}, tcp={})".format(
                        pose_str, maxPositionError, maxOrientationError, tcp_str
                    )
            
            prog_lines.append("def check_ik_program():")
            prog_lines.append("  has_solution = {}".format(ik_call))
            prog_lines.append("  socket_open(\"{}\", {})".format(local_ip, port))
            prog_lines.append("  socket_send_line(has_solution)")
            prog_lines.append("  socket_close()")
            prog_lines.append("end")
            prog_lines.append("check_ik_program()")
            
            prog = "\n".join(prog_lines)
            self.logger.info("Sending IK solution check program: %s", prog)
            
            # Send the program
            self.send_program(prog)
            
            # Wait for connection from robot
            self.logger.info("Waiting for robot to connect...")
            client_socket, addr = server_socket.accept()
            self.logger.info("Robot connected from %s", addr)
            
            # Receive the boolean result
            client_socket.settimeout(5.0)  # Increased from 2.0 for WSL
            data = b""
            
            # Read until we get a newline
            while b"\n" not in data:
                chunk = client_socket.recv(1024)
                if not chunk:
                    raise Exception("Connection closed before receiving solution check result")
                data += chunk
            
            # Extract the result
            line = data.split(b"\n", 1)[0]
            result_str = line.decode().strip().lower()
            
            # URScript returns "True" or "False" as strings
            has_solution = result_str == "true"
            
            client_socket.close()
            self.logger.info("IK solution check result: %s", has_solution)
            return has_solution
            
        except socket.timeout:
            self.logger.info("Timeout waiting for IK solution check result (WSL?)")
            raise Exception("Timeout waiting for inverse kinematics solution check result")
        except Exception as ex:
            self.logger.info("Error checking inverse kinematics solution: %s", ex)
            raise
        finally:
            if client_socket:
                try:
                    client_socket.close()
                except:
                    pass
            server_socket.close()

    def close(self):
        self._trystop = True
        self.join()
        
        # Close the socket
        try:
            self._s_secondary.close()
        except Exception as ex:
            self.logger.debug("Error closing socket: %s", ex)
