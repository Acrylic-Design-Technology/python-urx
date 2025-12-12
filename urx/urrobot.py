"""
Python library to control an UR robot through its TCP/IP interface
Documentation from universal robots:
http://support.universal-robots.com/URRobot/RemoteAccess
"""

import logging
import numbers

try:
    from collections.abc import Sequence
except ImportError:
    from collections import Sequence

from urx import urrtmon
from urx import ursecmon

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"


class RobotException(Exception):
    pass


class URRobot(object):
    """
    Python interface to socket interface of UR robot.
    programs are send to port 3002
    data is read from secondary interface(10Hz?) and real-time interface(125Hz) (called Matlab interface in documentation)
    Since parsing the RT interface uses som CPU, and does not support all robots versions, it is disabled by default
    The RT interfaces is only used for the get_force related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """

    def __init__(self, host, use_rt=False, urFirm=None):
        # self.logger = logging.getLogger("urx")
        self.logger = logging.getLogger('URX Logger')
        self.host = host
        self.urFirm = urFirm
        self.csys = None

        self.logger.debug("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz

        self.rtmon = None
        if use_rt:
            self.rtmon = self.get_realtime_monitor()
        # precision of joint movem used to wait for move completion
        # the value must be conservative! otherwise we may wait forever
        self.joinEpsilon = 0.01
        # It seems URScript is  limited in the character length of floats it accepts
        self.max_float_length = 6  # FIXME: check max length!!!

        self.secmon.wait()  # make sure we get data from robot before letting clients access our methods

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (
            self.host,
            self.secmon.get_all_data()["RobotModeData"],
        )

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def is_running(self):
        """
        Return True if robot is running (not
        necessary running a program, it might be idle)
        """
        return self.secmon.running

    def is_program_running(self):
        """
        check if program is running.
        Warning!!!!!:  After sending a program it might take several 10th of
        a second before the robot enters the running state
        """
        return self.secmon.is_program_running()

    def send_program(self, prog):
        """
        send a complete program using urscript to the robot
        the program is executed immediatly and any runnning
        program is interrupted
        """
        self.logger.info("Sending program: " + prog)
        self.secmon.send_program(prog)

    def get_tcp_force(self, wait=True):
        """
        return measured force in TCP
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getTCFForce(wait)

    def get_force(self, wait=True):
        """
        length of force vector returned by get_tcp_force
        if wait==True, waits for next packet before returning
        """
        tcpf = self.get_tcp_force(wait)
        force = 0
        for i in tcpf:
            force += i**2
        return force**0.5

    def get_joint_temperature(self, wait=True):
        """
        return measured joint temperature
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getJOINTTemperature(wait)

    def get_joint_voltage(self, wait=True):
        """
        return measured joint voltage
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getJOINTVoltage(wait)

    def get_joint_current(self, wait=True):
        """
        return measured joint current
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getJOINTCurrent(wait)

    def get_main_voltage(self, wait=True):
        """
        return measured Safety Control Board: Main voltage
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getMAINVoltage(wait)

    def get_robot_voltage(self, wait=True):
        """
        return measured Safety Control Board: Robot voltage (48V)
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getROBOTVoltage(wait)

    def get_robot_current(self, wait=True):
        """
        return measured Safety Control Board: Robot current
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getROBOTCurrent(wait)

    def get_all_rt_data(self, wait=True):
        """
        return all data parsed from robot real-time interace as a dict
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getALLData(wait)

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        # Format floats explicitly to avoid scientific notation
        prog = "set_tcp(p[{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}])".format(*tcp)
        self.send_program(prog)

    def set_payload(self, weight, cog=None):
        """
        set payload in Kg
        cog is a vector x,y,z
        if cog is not specified, then tool center point is used
        """
        if cog:
            # Format floats explicitly to avoid scientific notation
            prog = "set_payload({:.6f}, ({:.6f},{:.6f},{:.6f}))".format(weight, *cog)
        else:
            prog = "set_payload({:.6f})".format(weight)
        self.send_program(prog)

    def set_gravity(self, vector):
        """
        set direction of gravity
        """
        prog = "set_gravity(%s)" % list(vector)
        self.send_program(prog)

    def send_message(self, msg):
        """
        send message to the GUI log tab on the robot controller
        """
        prog = "textmsg(%s)" % msg
        self.send_program(prog)

    def set_digital_out(self, output, val):
        """
        set digital output. val is a bool
        """
        if val in (True, 1):
            val = "True"
        else:
            val = "False"
        self.send_program("digital_out[%s]=%s" % (output, val))

    def get_analog_inputs(self):
        """
        get analog input
        """
        return self.secmon.get_analog_inputs()

    def get_analog_in(self, nb, wait=False):
        """
        get analog input
        """
        return self.secmon.get_analog_in(nb, wait=wait)

    def get_digital_in_bits(self):
        """
        get digital output
        """
        return self.secmon.get_digital_in_bits()

    def get_digital_in(self, nb, wait=False):
        """
        get digital output
        """
        return self.secmon.get_digital_in(nb, wait)

    def get_digital_out(self, val, wait=False):
        """
        get digital output
        """
        return self.secmon.get_digital_out(val, wait=wait)

    def get_digital_out_bits(self, wait=False):
        """
        get digital output as a byte
        """
        return self.secmon.get_digital_out_bits(wait=wait)

    def set_analog_out(self, output, val):
        """
        set analog output, val is a float
        """
        prog = "set_analog_out(%s, %s)" % (output, val)
        self.send_program(prog)

    def set_tool_voltage(self, val):
        """
        set voltage to be delivered to the tool, val is 0, 12 or 24
        """
        prog = "set_tool_voltage(%s)" % (val)
        self.send_program(prog)

    def _wait_for_move(self, target, threshold=None, timeout=60, joints=False):
        """
        wait for a move to complete. Unfortunately there is no good way to know when a move has finished
        so for every received data from robot we compute a dist equivalent and when it is lower than
        'threshold' we return.
        if threshold is not reached within timeout, an exception is raised
        """
        self.logger.debug(
            "Waiting for move completion using threshold %s and target %s",
            threshold,
            target,
        )
        start_dist = self._get_dist(target, joints)
        if threshold is None:
            threshold = (
                start_dist * 0.8
            )  # FRED: THIS IS SUPPOSED TO BE *0.8 BUT IT BREAKS SOME MOVEMENTS WITH ANGLES
            if threshold < 0.001:  # roboten precision is limited
                threshold = 0.001
            self.logger.debug("No threshold set, setting it to %s", threshold)
        count = 0
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            dist = self._get_dist(target, joints)
            self.logger.debug(
                "distance to target is: %s, target dist is %s", dist, threshold
            )
            if not self.secmon.is_program_running():
                if dist < threshold:
                    self.logger.debug(
                        "we are threshold(%s) close to target, move has ended",
                        threshold,
                    )
                    return
                count += 1
                if count > timeout * 10:
                    raise RobotException(
                        "Goal not reached but no program has been running for {} seconds. dist is {}, threshold is {}, target is {}, current pose is {}".format(
                            timeout, dist, threshold, target, URRobot.getl(self)
                        )
                    )
            else:
                count = 0

    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        if(target[5] == 0): #Added by Fred
            if(((((-target[3]-pose[3])+(-target[4]-pose[4]))**2)<(((target[3]-pose[3])+(target[4]-pose[4]))**2))**0.5):
                target[3] = -target[3]
                target[4] = -target[4]
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        # for i in range(3, 6): #Commented by Fred. This breaks on equivalent vectors of different numbers, which is why the above addition exists but it doesnt cover all cases which is why this is commented out for now
        #     dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist**0.5

    def _get_joints_dist(self, target):
        joints = self.getj(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
        return dist**0.5

    def getj(self, wait=False):
        """
        get joints position
        """
        jts = self.secmon.get_joint_data(wait)
        return [
            jts["q_actual0"],
            jts["q_actual1"],
            jts["q_actual2"],
            jts["q_actual3"],
            jts["q_actual4"],
            jts["q_actual5"],
        ]

    def get_inverse_kin(self, pose, qnear=None, maxPositionError=1e-10,
                        maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Calculate inverse kinematics for a given pose.
        Returns joint positions that achieve the specified tool pose.
        
        Parameters:
            pose: tool pose as list [x, y, z, rx, ry, rz]
            qnear: list of joint positions for preferred solution (optional)
            maxPositionError: maximum allowed position error (default 1e-10)
            maxOrientationError: maximum allowed orientation error (default 1e-10)
            tcp: tcp offset pose or 'active_tcp' string (default 'active_tcp')
        
        Returns:
            list of 6 joint positions [j0, j1, j2, j3, j4, j5]
        
        Example:
            pose = [0.1, 0.2, 0.2, 0, 3.14, 0]
            joints = robot.get_inverse_kin(pose)
            
            # Get solution near current position
            current_joints = robot.getj()
            joints = robot.get_inverse_kin(pose, qnear=current_joints)
        
        See URScript get_inverse_kin() documentation for details.
        """
        return self.secmon.get_inverse_kin(pose, qnear, maxPositionError, 
                                           maxOrientationError, tcp)

    def get_inverse_kin_has_solution(self, pose, qnear=None, maxPositionError=1e-10,
                                       maxOrientationError=1e-10, tcp='active_tcp'):
        """
        Check if inverse kinematics has a solution for a given pose.
        Returns boolean (True) or (False).
        This can be used to avoid the runtime exception of get_inverse_kin
        when no solution exists.
        
        Parameters:
            pose: tool pose as list [x, y, z, rx, ry, rz]
            qnear: list of joint positions for preferred solution (optional)
            maxPositionError: maximum allowed position error (default 1e-10)
            maxOrientationError: maximum allowed orientation error (default 1e-10)
            tcp: tcp offset pose or 'active_tcp' string (default 'active_tcp')
        
        Returns:
            bool: True if get_inverse_kin has a solution, False otherwise
        
        Example:
            pose = [0.1, 0.2, 0.2, 0, 3.14, 0]
            if robot.get_inverse_kin_has_solution(pose):
                joints = robot.get_inverse_kin(pose)
                robot.movej(joints)
            else:
                print("No IK solution available for this pose")
            
            # Check solution near current position
            current_joints = robot.getj()
            if robot.get_inverse_kin_has_solution(pose, qnear=current_joints):
                joints = robot.get_inverse_kin(pose, qnear=current_joints)
        
        See URScript get_inverse_kin_has_solution() documentation for details.
        """
        return self.secmon.get_inverse_kin_has_solution(pose, qnear, maxPositionError,
                                                          maxOrientationError, tcp)

    def speedx(self, command, velocities, acc, min_time):
        # Format floats explicitly to avoid scientific notation
        vels = ["{:.6f}".format(i) for i in velocities]
        prog = "{}([{},{},{},{},{},{}], {:.6f}, {:.6f})".format(command, *vels, acc, min_time)
        self.send_program(prog)

    def movej(
        self,
        joints,
        acc=0.1,
        vel=0.05,
        wait=True,
        relative=False,
        threshold=None,
        prefix="",
    ):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        prog = self._format_move("movej", joints, acc, vel, prefix=prefix)
        self.send_program(prog)
        if wait:
            if prefix == "":
                self._wait_for_move(joints[:6], threshold=threshold, joints=True)
            if prefix == "p":
                self._wait_for_move(joints[:6], threshold=threshold, joints=False)
            return URRobot.getj(self)

    def movel(
        self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None, type="pose"
    ):
        """
        Send a movel command to the robot. See URScript documentation.
        type: "pose" for Cartesian pose (default), "joints" for joint positions
        """
        # Map type to prefix for URScript command generation
        prefix = "p" if type == "pose" else ""
        
        return URRobot.movex(
            self,
            "movel",
            tpose,
            acc=acc,
            vel=vel,
            wait=wait,
            relative=relative,
            threshold=threshold,
            prefix=prefix,
        )

    def movep(
        self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None
    ):
        """
        Send a movep command to the robot. See URScript documentation.
        """
        return URRobot.movex(
            self,
            "movep",
            tpose,
            acc=acc,
            vel=vel,
            wait=wait,
            relative=relative,
            threshold=threshold,
        )

    def servoc(
        self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None
    ):
        """
        Send a servoc command to the robot. See URScript documentation.
        """
        return self.movex(
            "servoc",
            tpose,
            acc=acc,
            vel=vel,
            wait=wait,
            relative=relative,
            threshold=threshold,
        )

    def servoj(
        self,
        tjoints,
        acc=0.01,
        vel=0.01,
        t=0.1,
        lookahead_time=0.2,
        gain=100,
        wait=True,
        relative=False,
        threshold=None,
    ):
        """
        Send a servoj command to the robot. See URScript documentation.
        """
        if relative:
            l = self.getj()
            tjoints = [v + l[i] for i, v in enumerate(tjoints)]
        prog = self._format_servo(
            "servoj",
            tjoints,
            acc=acc,
            vel=vel,
            t=t,
            lookahead_time=lookahead_time,
            gain=gain,
        )
        self.send_program(prog)
        if wait:
            self._wait_for_move(tjoints[:6], threshold=threshold, joints=True)
            return self.getj()

    def _format_servo(
        self,
        command,
        tjoints,
        acc=0.01,
        vel=0.01,
        t=0.1,
        lookahead_time=0.2,
        gain=100,
        prefix="",
    ):
        # Format floats explicitly to avoid scientific notation
        tjoints_formatted = ["{:.6f}".format(i) for i in tjoints]
        return "{}({}[{},{},{},{},{},{}], a={:.6f}, v={:.6f}, t={:.6f}, lookahead_time={:.6f}, gain={})".format(
            command, prefix, *tjoints_formatted, acc, vel, t, lookahead_time, int(gain)
        )

    def _format_move(self, command, tpose, acc, vel, radius=0, prefix=""):
        # Format floats explicitly to avoid scientific notation
        tpose_formatted = ["{:.6f}".format(i) for i in tpose]
        return "{}({}[{},{},{},{},{},{}], a={:.6f}, v={:.6f}, r={:.6f})".format(
            command, prefix, *tpose_formatted, acc, vel, radius
        )

    def movex(
        self,
        command,
        tpose,
        acc=0.01,
        vel=0.01,
        wait=True,
        relative=False,
        threshold=None,
        prefix="p",
    ):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        prefix: "p" for pose (default), "" for joint positions
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        prog = self._format_move(command, tpose, acc, vel, prefix=prefix)
        self.send_program(prog)
        if wait:
            if prefix == "":
                self._wait_for_move(tpose[:6], threshold=threshold, joints=True)
            else:
                self._wait_for_move(tpose[:6], threshold=threshold)
            return URRobot.getl(self)

    def getl(self, wait=False, _log=True):
        """
        get TCP position
        """
        pose = self.secmon.get_cartesian_info(wait)
        if pose:
            pose = [pose["X"], pose["Y"], pose["Z"], pose["Rx"], pose["Ry"], pose["Rz"]]
        if _log:
            self.logger.debug("Received pose from robot: %s", pose)
        return pose

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        # Format floats explicitly to avoid scientific notation
        pose_via_fmt = ["{:.6f}".format(i) for i in pose_via]
        pose_to_fmt = ["{:.6f}".format(i) for i in pose_to]
        prog = "movec(p[{},{},{},{},{},{}], p[{},{},{},{},{},{}], a={:.6f}, v={:.6f}, r=0)".format(
            *pose_via_fmt, *pose_to_fmt, acc, vel
        )
        self.send_program(prog)
        if wait:
            self._wait_for_move(pose_to, threshold=threshold)
            return self.getl()

    def movejs(
        self,
        joint_positions_list,
        acc=0.01,
        vel=0.01,
        radius=0.01,
        wait=True,
        threshold=None,
    ):
        """
        Concatenate several movej commands and applies a blending radius
        joint_positions_list is a list of joint_positions.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        return URRobot.movexs(
            self,
            "movej",
            joint_positions_list,
            acc,
            vel,
            radius,
            wait,
            threshold=threshold,
        )

    def movels(
        self, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None
    ):
        """
        Concatenate several movel commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        return URRobot.movexs(
            self, "movel", pose_list, acc, vel, radius, wait, threshold=threshold
        )

    def moveps(
        self, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None
    ):
        # NOT NATIVE TO URX
        """
        Concatenate several movep commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        return URRobot.movexs(
            self, "movep", pose_list, acc, vel, radius, wait, threshold=threshold
        )

    def movexs(
        self,
        command,
        pose_list,
        acc=0.01,
        vel=0.01,
        radius=0.01,
        wait=True,
        threshold=None,
    ):
        """
        Concatenate several movex commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        header = "def myProg():\n"
        end = "end\n"
        prog = header
        # Check if 'vel' is a single number or a sequence.
        if isinstance(vel, numbers.Number):
            # Make 'vel' a sequence
            vel = len(pose_list) * [vel]
        elif not isinstance(vel, Sequence):
            raise RobotException('movexs: "vel" must be a single number or a sequence!')
        # Check for adequate number of speeds
        if len(vel) != len(pose_list):
            raise RobotException(
                'movexs: "vel" must be a number or a list '
                + 'of numbers the same length as "pose_list"!'
            )
        # Check if 'acc' is a single number or a sequence.
        if isinstance(acc, numbers.Number):
            # Make 'vel' a sequence
            acc = len(pose_list) * [acc]
        elif not isinstance(acc, Sequence):
            raise RobotException('movexs: "acc" must be a single number or a sequence!')
        # Check for adequate number of speeds
        if len(acc) != len(pose_list):
            raise RobotException(
                'movexs: "acc" must be a number or a list '
                + 'of numbers the same length as "pose_list"!'
            )
        # Check if 'radius' is a single number.
        if isinstance(radius, numbers.Number):
            # Make 'radius' a sequence
            radius = len(pose_list) * [radius]
        elif not isinstance(radius, Sequence):
            raise RobotException(
                'movexs: "radius" must be a single number or a sequence!'
            )
        # Ensure that last pose a stopping pose.
        radius[-1] = 0.0
        # Require adequate number of radii.
        if len(radius) != len(pose_list):
            raise RobotException(
                'movexs: "radius" must be a number or a list '
                + 'of numbers the same length as "pose_list"!'
            )
        prefix = ""
        if command in ["movel", "movec", "movep"]:
            prefix = "p"
        for idx, pose in enumerate(pose_list):
            prog += (
                self._format_move(
                    command, pose, acc[idx], vel[idx], radius[idx], prefix=prefix
                )
                + "\n"
            )
        prog += end
        self.send_program(prog)
        if wait:
            if command == "movel" or command == "movep":
                self._wait_for_move(
                    target=pose_list[-1], threshold=threshold, joints=False
                )
            elif command == "movej":
                self._wait_for_move(
                    target=pose_list[-1], threshold=threshold, joints=True
                )
            return URRobot.getl(self)

    def movebatch(
        self,
        command_list,
        pose_list,
        acc=0.01,
        vel=0.01,
        radius=0.01,
        type_list=None,
        wait=True,
        threshold=None,
    ):
        """
        Concatenate several move commands of different types and applies blending radii.
        
        This method allows mixing movel, movep, movej, and movec commands in a single program.
        Each command type can have its own pose/joints, velocity, acceleration, and blend radius.
        
        Args:
            command_list: List of command types (e.g., ["movel", "movep", "movej", "movec"])
            pose_list: List of poses or joint positions. For movec, provide (via_pose, to_pose) as a tuple
            acc: Single acceleration value or list of accelerations (one per command)
            vel: Single velocity value or list of velocities (one per command)
            radius: Single blend radius value or list of radii (one per command)
            type_list: Optional list specifying "pose" or "joints" for each command. 
                      Only relevant for movel commands. None entries default to "pose".
            wait: Wait for move completion
            threshold: Distance threshold for move completion detection
            
        Example:
            robot.movebatch(
                command_list=["movel", "movep", "movep", "movel"],
                pose_list=[joints1, pose2, pose3, pose4],
                type_list=["joints", "pose", "pose", "pose"],
                vel=[0.1, 0.2, 0.2, 0.1],
                acc=[0.5, 0.3, 0.3, 0.5],
                radius=[0.01, 0.02, 0.02, 0.0]
            )
        """
        # Validate command_list
        if not isinstance(command_list, Sequence) or isinstance(command_list, str):
            raise RobotException('movebatch: "command_list" must be a list of command strings!')
        
        if len(command_list) == 0:
            raise RobotException('movebatch: "command_list" cannot be empty!')
        
        # Validate pose_list length matches command_list
        if len(pose_list) != len(command_list):
            raise RobotException(
                f'movebatch: "pose_list" length ({len(pose_list)}) must match '
                f'"command_list" length ({len(command_list)})!'
            )
        
        # Check if 'vel' is a single number or a sequence
        if isinstance(vel, numbers.Number):
            vel = len(command_list) * [vel]
        elif not isinstance(vel, Sequence):
            raise RobotException('movebatch: "vel" must be a single number or a sequence!')
        
        if len(vel) != len(command_list):
            raise RobotException(
                f'movebatch: "vel" must be a number or a list of numbers '
                f'the same length as "command_list"!'
            )
        
        # Check if 'acc' is a single number or a sequence
        if isinstance(acc, numbers.Number):
            acc = len(command_list) * [acc]
        elif not isinstance(acc, Sequence):
            raise RobotException('movebatch: "acc" must be a single number or a sequence!')
        
        if len(acc) != len(command_list):
            raise RobotException(
                f'movebatch: "acc" must be a number or a list of numbers '
                f'the same length as "command_list"!'
            )
        
        # Check if 'radius' is a single number or a sequence
        if isinstance(radius, numbers.Number):
            radius = len(command_list) * [radius]
        elif not isinstance(radius, Sequence):
            raise RobotException('movebatch: "radius" must be a single number or a sequence!')
        
        if len(radius) != len(command_list):
            raise RobotException(
                f'movebatch: "radius" must be a number or a list of numbers '
                f'the same length as "command_list"!'
            )
        
        # Ensure last radius is 0 (stopping pose)
        radius = list(radius)  # Convert to list if tuple
        radius[-1] = 0.0
        
        # Setup type_list if provided
        if type_list is None:
            type_list = ["pose"] * len(command_list)
        elif not isinstance(type_list, Sequence):
            raise RobotException('movebatch: "type_list" must be a list!')
        
        if len(type_list) != len(command_list):
            raise RobotException(
                f'movebatch: "type_list" length must match "command_list" length!'
            )
        
        # Build the program
        header = "def myProg():\n"
        end = "end\n"
        prog = header
        
        # Track last command for wait logic
        last_command = command_list[-1]
        last_pose = pose_list[-1]
        last_is_joints = False
        
        for idx, command in enumerate(command_list):
            pose = pose_list[idx]
            cmd_type = type_list[idx] if type_list[idx] is not None else "pose"
            
            # Check if we need to insert a stopl before this command
            # Insert stopl when transitioning between movel and movep
            if idx > 0:
                prev_command = command_list[idx-1]
                if (prev_command == "movep" and command == "movel") or \
                   (prev_command == "movel" and command == "movep"):
                    prog += "stopl(1)\n"
            
            # Determine prefix based on command type and type_list
            if command == "movel":
                # movel can use either pose or joint coordinates
                prefix = "" if cmd_type == "joints" else "p"
                if idx == len(command_list) - 1:
                    last_is_joints = (cmd_type == "joints")
            elif command == "movep":
                prefix = "p"
            elif command == "movej":
                prefix = ""
                if idx == len(command_list) - 1:
                    last_is_joints = True
            elif command == "movec":
                # movec requires two poses: via and to
                # Expect pose to be a tuple of (via_pose, to_pose)
                if not isinstance(pose, (tuple, list)) or len(pose) != 2:
                    raise RobotException(
                        f'movebatch: For movec command at index {idx}, pose must be '
                        f'a tuple/list of (via_pose, to_pose)!'
                    )
                # Format floats explicitly to avoid scientific notation
                via_pose_fmt = ["{:.6f}".format(i) for i in pose[0]]
                to_pose_fmt = ["{:.6f}".format(i) for i in pose[1]]
                prog += "movec(p[{},{},{},{},{},{}], p[{},{},{},{},{},{}], a={:.6f}, v={:.6f}, r={:.6f})\n".format(
                    *via_pose_fmt, *to_pose_fmt, acc[idx], vel[idx], radius[idx]
                )
                continue  # Skip the normal _format_move call
            else:
                raise RobotException(
                    f'movebatch: Unknown command "{command}" at index {idx}. '
                    f'Supported commands: movel, movep, movej, movec'
                )
            
            # Generate move command using existing helper
            prog += (
                self._format_move(
                    command, pose, acc[idx], vel[idx], radius[idx], prefix=prefix
                )
                + "\n"
            )
        
        prog += end
        self.send_program(prog)
        
        if wait:
            # Wait for the last movement to complete
            if last_command == "movec":
                # For movec, last_pose is a tuple, use the 'to' pose
                self._wait_for_move(
                    target=last_pose[1], threshold=threshold, joints=False
                )
            elif last_command in ["movel", "movep"]:
                self._wait_for_move(
                    target=last_pose, threshold=threshold, joints=last_is_joints
                )
            elif last_command == "movej":
                self._wait_for_move(
                    target=last_pose, threshold=threshold, joints=True
                )
            return URRobot.getl(self)

    def stopl(self, acc=0.5):
        self.send_program("stopl(%s)" % acc)

    def stopj(self, acc=1.5):
        self.send_program("stopj(%s)" % acc)

    def stop(self):
        self.stopj()

    def close(self):
        """
        close connection to robot and stop internal thread
        """
        self.logger.info("Closing sockets to robot")
        self.secmon.close()
        if self.rtmon:
            self.rtmon.stop()
    
    def get_connection_stats(self):
        """
        Return dictionary of connection health metrics.
        
        Returns:
            dict: Connection statistics with keys:
                - 'state_reader': StateReaderSocket statistics
                - 'command_socket': CommandSocket statistics
        
        Example:
            stats = robot.get_connection_stats()
            print(f"State reader connects: {stats['state_reader']['total_connects']}")
            print(f"Command socket timeouts: {stats['command_socket']['timeout_count']}")
        """
        return {
            'state_reader': self.secmon._state_reader.get_stats(),
            'command_socket': self.secmon._cmd_socket.get_stats(),
        }
    
    def reset_connection_stats(self):
        """
        Reset all connection metrics counters.
        
        Useful for monitoring connection health over specific time periods
        or after recovering from connection issues.
        """
        self.secmon._state_reader.reset_stats()
        self.secmon._cmd_socket.reset_stats()

    def set_freedrive(self, val, timeout=60):
        """
        set robot in freedrive/backdrive mode where an operator can jog
        the robot to wished pose.

        Freedrive will timeout at 60 seconds.
        """
        if val:
            self.send_program(
                "def myProg():\n\tfreedrive_mode()\n\tsleep({})\nend".format(timeout)
            )
        else:
            # This is a non-existant program, but running it will stop freedrive
            self.send_program("def myProg():\n\tend_freedrive_mode()\nend")

    def set_simulation(self, val):
        if val:
            self.send_program("set sim")
        else:
            self.send_program("set real")

    def get_realtime_monitor(self):
        """
        return a pointer to the realtime monitor object
        usefull to track robot position for example
        """
        if not self.rtmon:
            self.logger.info("Opening real-time monitor socket")
            self.rtmon = urrtmon.URRTMonitor(
                self.host, self.urFirm
            )  # som information is only available on rt interface
            self.rtmon.start()
        self.rtmon.set_csys(self.csys)
        return self.rtmon

    def translate(self, vect, acc=0.01, vel=0.01, wait=True, command="movel"):
        """
        move tool in base coordinate, keeping orientation
        """
        p = self.getl()
        p[0] += vect[0]
        p[1] += vect[1]
        p[2] += vect[2]
        return self.movex(command, p, vel=vel, acc=acc, wait=wait)

    def up(self, z=0.05, acc=0.01, vel=0.01):
        """
        Move up in csys z
        """
        p = self.getl()
        p[2] += z
        self.movel(p, acc=acc, vel=vel)

    def down(self, z=0.05, acc=0.01, vel=0.01):
        """
        Move down in csys z
        """
        self.up(-z, acc, vel)